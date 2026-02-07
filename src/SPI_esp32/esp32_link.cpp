#include "esp32_link.hpp"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include <cstring>

LOG_MODULE_REGISTER(esp32_link, LOG_LEVEL_INF);

/* ========= Devicetree 节点（沿用你之前 debug main 里的同一套） ========= */
#define NODE_BQ25120A   DT_NODELABEL(bq25120a)
#define NODE_LS_3V3     DT_CHILD(NODE_BQ25120A, load_switch)
#define NODE_LS_SD      DT_NODELABEL(load_switch_sd)
#define NODE_SD_STATE   DT_NODELABEL(sd_state)

/* OpenEarable 的 SD 卡 SPI 总线：你之前已验证是 spi4 */
#define SPI_NODE        DT_NODELABEL(spi4)

/* ========= GPIO specs ========= */
static const gpio_dt_spec ls_3v3_en = GPIO_DT_SPEC_GET(NODE_LS_3V3, enable_gpios);
static const gpio_dt_spec ls_sd_en  = GPIO_DT_SPEC_GET(NODE_LS_SD,  enable_gpios);
static const gpio_dt_spec sd_state  = GPIO_DT_SPEC_GET(NODE_SD_STATE, gpios);

/* ========= SPI device ========= */
static const device* const spi_dev = DEVICE_DT_GET(SPI_NODE);

/* ========= CS 引脚：优先从 spi4 的 cs-gpios 拿（通常就是 gpio0.11），否则回退 ========= */
#if DT_NODE_HAS_PROP(SPI_NODE, cs_gpios)
static const gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET_BY_IDX(SPI_NODE, cs_gpios, 0);
#else
/* 如果你的板子 DTS 没写 cs-gpios，就回退到 gpio0 pin11（你之前描述过） */
#define FALLBACK_GPIO0  DT_NODELABEL(gpio0)
static const device* const gpio0 = DEVICE_DT_GET(FALLBACK_GPIO0);
static gpio_dt_spec cs_gpio = {
    .port = gpio0,
    .pin  = 11,
    .dt_flags = 0,
};
#endif

/* ========= SPI 配置 ========= */
static spi_config spi_cfg;

/* 互斥：避免多线程同时打 SPI（以后你加 BLE 指令时很容易踩这个坑） */
K_MUTEX_DEFINE(g_spi_lock);

static bool g_inited = false;

/* 物理电平控制 CS：0=物理低，1=物理高（不走 dt 的 active_low 语义） */
static inline void cs_set_phys(int level)
{
    /* gpio_pin_set_raw: 直接写物理电平，不受 GPIO_ACTIVE_LOW 影响 */
    (void)gpio_pin_set_raw(cs_gpio.port, cs_gpio.pin, level);
}

static int ensure_ready_common()
{
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("spi4 not ready");
        return -ENODEV;
    }

    if (!device_is_ready(ls_3v3_en.port)) {
        LOG_ERR("ls_3v3_en port not ready");
        return -ENODEV;
    }
    if (!device_is_ready(ls_sd_en.port)) {
        LOG_ERR("ls_sd_en port not ready");
        return -ENODEV;
    }
    if (!device_is_ready(sd_state.port)) {
        LOG_ERR("sd_state port not ready");
        return -ENODEV;
    }

    if (!device_is_ready(cs_gpio.port)) {
        LOG_ERR("cs gpio port not ready");
        return -ENODEV;
    }

    return 0;
}

static void spi_config_init()
{
    std::memset(&spi_cfg, 0, sizeof(spi_cfg));

    /* 1MHz：大于 125kHz 下限，且对飞线非常友好 */
    spi_cfg.frequency = 1000000U;

    /* 完全对齐你 DK 验证代码：不额外加 SPI_MODE_0（默认就是 Mode0） */
    spi_cfg.operation = SPI_OP_MODE_MASTER
                      | SPI_WORD_SET(8)
                      | SPI_TRANSFER_MSB;

    spi_cfg.slave = 0;

    /* 关键：不让驱动自动管 CS（我们手动拉脚）——你的环境里 cs 是 struct，不是指针 */
    spi_cfg.cs.gpio.port = nullptr;
    spi_cfg.cs.gpio.pin = 0;
    spi_cfg.cs.gpio.dt_flags = 0;
    spi_cfg.cs.delay = 0;
}

int esp32_link_set_power(bool on)
{
    int ret = ensure_ready_common();
    if (ret) {
        return ret;
    }

    /* 1) 3V3 总开关保持 ON（你的 debug 已验证这样能让底座电平转换工作） */
    ret = gpio_pin_configure_dt(&ls_3v3_en, GPIO_OUTPUT_ACTIVE);
    if (ret) {
        LOG_ERR("ls_3v3_en configure failed: %d", ret);
        return ret;
    }

    /* 2) SD 域开关（V_SD） */
    ret = gpio_pin_configure_dt(&ls_sd_en, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        LOG_ERR("ls_sd_en configure failed: %d", ret);
        return ret;
    }

    gpio_pin_set_dt(&ls_sd_en, on ? 1 : 0);

    /* 给电平转换器/供电一点稳定时间（飞线尤其需要） */
    k_sleep(K_MSEC(20));

    /* 读回确认（用于排除“有人又把 LS_SD 拉低了”） */
    int rb = gpio_pin_get_dt(&ls_sd_en);
    int st = gpio_pin_get_dt(&sd_state);
    LOG_INF("Power %s: LS_SD readback=%d | SD_STATE=%d", on ? "ON" : "OFF", rb, st);

    return 0;
}

int esp32_link_init(void)
{
    int ret = ensure_ready_common();
    if (ret) {
        return ret;
    }

    /* 配置 SD_STATE 为输入（只是辅助观测，不影响 SPI） */
    ret = gpio_pin_configure_dt(&sd_state, GPIO_INPUT);
    if (ret) {
        LOG_WRN("sd_state configure failed: %d (continue)", ret);
    }

    /* 配置 CS 引脚为输出，并且“物理拉高”为 idle */
    ret = gpio_pin_configure(cs_gpio.port, cs_gpio.pin, GPIO_OUTPUT_HIGH);
    if (ret) {
        LOG_ERR("CS configure failed: %d", ret);
        return ret;
    }
    cs_set_phys(1); /* idle high */

    /* 强制把 SD 域供电打开（否则你会出现：插上 ESP32 全 00 / 从机 timeout） */
    ret = esp32_link_set_power(true);
    if (ret) {
        return ret;
    }

    spi_config_init();

    g_inited = true;
    LOG_INF("esp32_link_init done (CS idle high, power ON, spi=1MHz mode0)");

    return 0;
}

int esp32_link_xfer(const uint8_t* tx, uint8_t* rx, size_t tx_len)
{
    if (!g_inited) {
        /* 允许用户忘了 init：这里兜底一下 */
        int r = esp32_link_init();
        if (r) {
            return r;
        }
    }

    if (tx == nullptr || tx_len == 0) {
        return -EINVAL;
    }

    /* 确保电源一直开着（即使别处误操作关了，也能拉回来） */
    (void)esp32_link_set_power(true);

    k_mutex_lock(&g_spi_lock, K_FOREVER);

    spi_buf txb{ .buf = (void*)tx, .len = tx_len };
    spi_buf_set txs{ .buffers = &txb, .count = 1 };

    spi_buf rxb{};
    spi_buf_set rxs{};

    if (rx) {
        std::memset(rx, 0, tx_len);
        rxb.buf = rx;
        rxb.len = tx_len;
        rxs.buffers = &rxb;
        rxs.count = 1;
    } else {
        rxs.buffers = nullptr;
        rxs.count = 0;
    }

    /* ===== 关键：CS 按“物理电平”拉低/拉高，完全对齐你 DK 验证逻辑 ===== */
    cs_set_phys(0);                 /* 物理拉低：选中从机 */
    k_busy_wait(2);                 /* 2us 小延迟，给从机/电平转换器反应时间 */

    int ret = spi_transceive(spi_dev, &spi_cfg, &txs, rx ? &rxs : nullptr);

    k_busy_wait(2);
    cs_set_phys(1);                 /* 物理拉高：取消选中 */

    k_mutex_unlock(&g_spi_lock);

    if (ret) {
        LOG_ERR("spi_transceive failed: %d", ret);
    }

    return ret;
}

int esp32_link_send(const uint8_t* tx, size_t tx_len)
{
    return esp32_link_xfer(tx, nullptr, tx_len);
}
