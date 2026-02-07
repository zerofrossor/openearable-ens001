#pragma once

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

int esp32_link_init(void);

/* 强制打开/关闭 SD 域供电（V_SD、电平转换器等） */
int esp32_link_set_power(bool on);

/* 全双工传输：发送 tx_len 字节，同时读回同长度到 rx（rx 可为 nullptr 表示不读） */
int esp32_link_xfer(const uint8_t* tx, uint8_t* rx, size_t tx_len);

/* 只写（不读回） */
int esp32_link_send(const uint8_t* tx, size_t tx_len);

#ifdef __cplusplus
}
#endif
