#include "spi_cmd_service.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "../SPI_esp32/esp32_link.hpp"  /* 按你的实际相对路径改一下 */

LOG_MODULE_REGISTER(spi_cmd_service, LOG_LEVEL_INF);

/*
 * 自定义 128-bit UUID（你也可以换成自己喜欢的，只要不重复）
 * Service UUID:  7A2B0001-8E5F-4B1A-9D3A-2E1F00000001
 * Char UUID:     7A2B0002-8E5F-4B1A-9D3A-2E1F00000002
 */
#define BT_UUID_SPI_CMD_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x7a2b0001, 0x8e5f, 0x4b1a, 0x9d3a, 0x2e1f00000001)

#define BT_UUID_SPI_CMD_CHAR_VAL \
	BT_UUID_128_ENCODE(0x7a2b0002, 0x8e5f, 0x4b1a, 0x9d3a, 0x2e1f00000002)

static struct bt_uuid_128 spi_cmd_svc_uuid = BT_UUID_INIT_128(BT_UUID_SPI_CMD_SERVICE_VAL);
static struct bt_uuid_128 spi_cmd_chr_uuid = BT_UUID_INIT_128(BT_UUID_SPI_CMD_CHAR_VAL);

/* 保存最近一次命令（1/2/3） */
static uint8_t g_last_cmd = 0;

/* 用 work 把 SPI 发送挪出蓝牙回调，避免阻塞 BT 线程 */
static struct k_work spi_cmd_work;

static void spi_cmd_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	uint8_t tx[4] = {0};
	uint8_t rx[4] = {0};

	switch (g_last_cmd) {
	case 1:
		tx[0]=0x11; tx[1]=0x22; tx[2]=0x33; tx[3]=0x44;
		break;
	case 2:
		tx[0]=0x22; tx[1]=0x33; tx[2]=0x44; tx[3]=0x55;
		break;
	case 3:
		tx[0]=0x33; tx[1]=0x44; tx[2]=0x55; tx[3]=0x77;
		break;
	default:
		LOG_WRN("Unknown cmd=%u (ignore)", g_last_cmd);
		return;
	}

	/* 确保 SPI 已初始化（如果你 main 里已经 init 过，这里也不会坏） */
	int ret = esp32_link_init();
	if (ret) {
		LOG_ERR("esp32_link_init failed: %d", ret);
		return;
	}

	ret = esp32_link_xfer(tx, rx, sizeof(tx));
	LOG_INF("SPI cmd=%u ret=%d tx=%02X %02X %02X %02X rx=%02X %02X %02X %02X",
		g_last_cmd, ret,
		tx[0], tx[1], tx[2], tx[3],
		rx[0], rx[1], rx[2], rx[3]);
}

/* 手机写入 characteristic 时触发 */
static ssize_t spi_cmd_write_cb(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len,
			       uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len < 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	/* 我们只取第一个字节：1/2/3 */
	const uint8_t *b = (const uint8_t *)buf;
	uint8_t cmd = b[0];

	if (cmd < 1 || cmd > 3) {
		LOG_WRN("SPI cmd write got %u (expect 1..3)", cmd);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	g_last_cmd = cmd;

	/* 立刻投递 work，异步去做 SPI */
	k_work_submit(&spi_cmd_work);

	LOG_INF("SPI cmd queued: %u", cmd);
	return len;
}

/* 定义 GATT Service + Characteristic（只需要 Write 即可） */
BT_GATT_SERVICE_DEFINE(spi_cmd_svc,
	BT_GATT_PRIMARY_SERVICE(&spi_cmd_svc_uuid),
	BT_GATT_CHARACTERISTIC(&spi_cmd_chr_uuid.uuid,
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE,
			       NULL, spi_cmd_write_cb, NULL),
);

int init_spi_cmd_service(void)
{
	k_work_init(&spi_cmd_work, spi_cmd_work_handler);
	LOG_INF("spi_cmd_service inited");
	return 0;
}
