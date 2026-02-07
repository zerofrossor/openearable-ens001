# OpenEarable (nRF5340) ↔ ESP32 SPI Link via SD-Base Pins

> 复用 OpenEarable 底座 SD/TF 接口的引脚与供电/电平转换电路，实现稳定 SPI 通信，并支持手机 BLE 写入指令触发 SPI 发送固定 payload。

---

## ✅ 项目状态（当前阶段已完成）

* [x] OpenEarable（nRF5340）作为 **SPI 主机**
* [x] ESP32 / ESP32-S3 作为 **SPI 从机**
* [x] 复用 SD 底座引脚（飞线）进行 SPI
* [x] 解决供电域未使能导致的“SPI 不通/全 0/全 FF”问题
* [x] 解决 CS 逻辑反相（`GPIO_ACTIVE_LOW`）导致的“从机一直未选中”问题
* [x] 新增 BLE GATT 写入特征：手机写入 `1/2/3` → 触发 SPI 发送三种 4-byte 指令
* [x] RTT 日志可观察：供电状态、SPI ret、TX/RX 数据

---

## 🎯 功能目标

在不破坏 OpenEarable 原有功能（BLE、传感器、LED、按钮等）的前提下，实现：

**手机 → BLE 写入 1 byte 指令 → OpenEarable 通过 SPI 发送 4 bytes 命令给 ESP32**

| 手机写入 (1 byte) | SPI TX (4 bytes, HEX) |
| ------------------- | ----------------------- |
| `0x01`        | `11 22 33 44`     |
| `0x02`        | `22 33 44 55`     |
| `0x03`        | `33 44 55 66`     |

ESP32 从机可打印接收到的数据；OpenEarable 侧 RTT 可打印 SPI 返回值与回包。

---

## 🧠 核心架构（数据流）

```
nRF Connect Mobile (Write 0x01/0x02/0x03)
        ↓
GATT Write Callback (不做耗时操作)
        ↓
k_work_submit()  (异步执行)
        ↓
esp32_link_xfer()  (SPI Transceive + 手动 CS)
        ↓
SPI wires via SD base pins (with level shifting + power domain)
        ↓
ESP32 SPI Slave receives 4 bytes and prints
```

---

## 🔧 关键经验总结（踩坑记录）

### 1) SD 底座引脚不是“裸 IO”，必须使能供电域（LS\_SD / V\_SD）

**最常见问题表现：**

* nRF 侧 `ret=0` 但 `rx=00 00 00 00`
* ESP32 侧一直 `timeout`
* 拔下时读到 `FF FF FF FF`（MISO 悬空上拉）

**根因：**
SD/TF 底座信号经过：

* Load Switch（供电开关）
* 电平转换/缓冲
* 供电域 V\_SD
  如果 ​**LS\_SD 没打开**​，外侧 SPI 线路“看起来有电平但实际不可用/不稳定”。

✅ 最终解决：
在 `esp32_link_init()` 内部 ​**强制 `esp32_link_set_power(true)`**​，确保：

* `LS_3V3` 打开
* `LS_SD` 打开（V\_SD 供电）
* 等待电平稳定（延时 20\~50ms）
* 可选：读回并打印状态，防止被其他模块关掉

---

### 2) CS 逻辑反相（`GPIO_ACTIVE_LOW`）会让从机永远没被选中

**典型表现：**

* SPI 调用不报错，但 ESP32 永远 timeout
* 或 nRF 侧读回全 0

**根因：**
Devicetree 的 `cs-gpios` 常配置为 `GPIO_ACTIVE_LOW`。
如果你用 `gpio_pin_set_dt()`（逻辑电平），可能会自动反相，导致你以为拉低 CS，实际上是拉高。

✅ 最终解决：
CS 采用 ​**物理电平控制**​：

* `gpio_pin_set_raw(cs_port, cs_pin, 0)` = 物理拉低（选中）
* `gpio_pin_set_raw(cs_port, cs_pin, 1)` = 物理拉高（释放）

这样行为完全对齐你在 nRF5340 DK 上验证成功的写法。

---

### 3) BLE 写入回调里不要直接跑 SPI（必须异步）

**原因：**
写入回调运行在蓝牙栈上下文中，做 SPI 可能阻塞 → 导致 BLE 不稳定。

✅ 最终解决：
使用 `k_work`：

* 回调：只校验/保存命令/submit work
* work handler：执行 `esp32_link_xfer()`

---

## 📁 工程改动清单（必须备份的内容）

> 路径以你当前工程结构为准（你已在工程里实现并验证通过）

### 1) SPI 封装模块（主机侧）

目录：`src/spi_esp32/`

* `esp32_link.hpp`
* `esp32_link.cpp`

**关键功能：**

* 初始化 SPI + CS
* 强制使能供电域（LS\_SD）
* SPI transceive
* CS 使用 `gpio_pin_set_raw`（物理电平）

---

### 2) BLE GATT 服务：写入 1/2/3 触发 SPI

目录：`src/bluetooth/gatt_services/`

* `spi_cmd_service.h`
* `spi_cmd_service.cpp`  ✅（必须是 `.cpp`，因为 include C++ 头）

**关键功能：**

* 自定义 128-bit UUID Service + Characteristic
* Characteristic 支持 Write / Write Without Response
* 写入 `0x01/0x02/0x03` → work handler 内调用 SPI 发送固定 payload
* RTT/LOG 打印发送与回包

---

### 3) CMakeLists.txt

确保编译进新文件：

```cmake
target_sources(app PRIVATE
  src/spi_esp32/esp32_link.cpp
  src/bluetooth/gatt_services/spi_cmd_service.cpp
)
```

> 注意：若使用 glob 或其他方式，也必须确认 `.cpp` 被包含，否则手机端看不到 service。

---

### 4) main 初始化（仅需 1 行）

在 main 的初始化阶段调用：

```cpp
ret = init_spi_cmd_service();
ERR_CHK(ret);
```

✅ 除此之外 ​**不需要改 main 的主流程**​。
（推荐：不要长期保留“每秒自动发 SPI”的测试线程，以免和 BLE 指令混淆）

---

## 🔌 接线与硬件注意事项（飞线必看）

* ✅ ​**必须共地**​：OpenEarable GND ↔ ESP32 GND
  否则可能出现 ret=0 但数据全乱/从机 timeout。
* ✅ 确认 V\_SD：
  * LS\_SD 打开时，底座相关引脚应能测到约 3V（你已验证）
  * 如果一直 0V，优先检查：LS\_SD 是否被拉高 / 是否被其他模块关掉
* ✅ MOSI/MISO/SCK/CS 不要接反
  最常见错误是 MOSI/MISO 对调，表现为：
  * ESP32 可能收到全 0 或完全收不到
  * nRF 侧读回全 0 或随机

---

## 🧪 运行与验证步骤

### A) OpenEarable（nRF5340）侧

1. ​**Pristine / Clean build**​（强烈建议）
2. 烧录（5340DK + J-Link）
3. 打开 RTT（SEGGER RTT Viewer / nRF Connect RTT）
4. 观察日志中是否出现：
   * `Power ON: LS_SD ...`
   * `SPI cmd queued: 1`
   * `SPI cmd=1 ret=0 tx=... rx=...`

---

### B) 手机（nRF Connect Mobile）

1. 连接 OpenEarable
2. 找到 **SPI CMD Service（自定义 UUID）**
3. 在 characteristic 里 Write：
   * 写 `01` → 触发 `11 22 33 44`
   * 写 `02` → 触发 `22 33 44 55`
   * 写 `03` → 触发 `33 44 55 66`

---

### C) ESP32（SPI 从机）

* 串口应打印收到的 4 字节数据
* 若持续 `timeout`：
  1. 检查共地
  2. 检查 CS 是否真拉低
  3. 检查 LS\_SD / V\_SD 是否确实供电
  4. 检查线序

---

## 🧯 常见问题（快速排障）

### 1) nRF 侧 `ret=0` 但 `rx=00 00 00 00`，ESP32 侧 timeout

优先排查顺序：

1. CS 是否物理拉低（是否用了 `gpio_pin_set_raw`）
2. LS\_SD / V\_SD 是否一直上电（是否被其他模块关掉）
3. MOSI/MISO 是否接反
4. 是否共地

---

### 2) 拔下 ESP32 时读到 FF，插上变成 00

* FF：MISO 悬空上拉（正常现象）
* 00：MISO 被外侧拉低/钳位，但从机可能没被选中
  重点看：CS 与供电域是否正确

---

### 3) 编译报错 `fatal error: cstddef: No such file or directory`

原因：`.c` 文件用 C 编译器（`-std=c99`）编译，不能 include C++ 标准头。
解决：将 BLE service 改为 `.cpp`，或提供 C 版桥接头（不推荐）。

---

## 🧾 环境信息（备份必须留存）

* NCS：v3.0.1（你日志中可见）
* Zephyr：v4.0.99-ncs1-1（你日志中可见）
* 工具：nRF Connect for VS Code / west / sysbuild
* 调试：SEGGER RTT（J-Link）

---

## 🚀 后续扩展建议（可选）

* 把 `1/2/3` 扩展为结构化命令（opcode + payload）
* 增加 Notify：把 ESP32 回包通过 BLE notify 发回手机
* 加队列：支持连续命令排队发送
* 与按钮/传感器事件联动：例如按键触发 SPI 命令

---

## 📌 维护建议（强烈推荐）

* 复制整个工程目录做快照：`openearable_spi_backup_YYYYMMDD/`
* README 放根目录：`README.md`
* 附上：
  * 飞线照片
  * 你最终测到的 V\_SD 波形/电压
  * 10 行 RTT “成功通信”日志

