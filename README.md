OpenEarable 2.2.x：复用 SD 卡底座引脚实现 SPI ↔ ESP32 通信（备份说明与使用手册）
1. 目标与当前阶段成果
1.1 目标

在 不破坏 OpenEarable 现有功能（BLE、传感器、LED、按钮等） 的前提下，复用 OpenEarable 底座 SD/TF 卡接口的引脚与电平转换电路，实现：

nRF5340（OpenEarable）作为 SPI 主机

ESP32/ESP32-S3 作为 SPI 从机

通过手机（nRF Connect Mobile 等）写入指令 1 / 2 / 3，触发 nRF5340 通过 SPI 发送三种固定 payload：

手机写入（1 byte）	SPI 发送（4 bytes, HEX）
0x01	11 22 33 44
0x02	22 33 44 55
0x03	33 44 55 66

同时，SPI 返回值/接收数据在 RTT/LOG 中可见，便于调试与验证。

2. 总体架构（你现在做成了什么）
2.1 数据流（最关键）

手机 → BLE GATT Write → nRF5340 写入回调 → k_work 异步执行 → esp32_link_xfer() → SPI 总线 → ESP32 SPI Slave 接收并打印

2.2 为什么要用 k_work（关键经验）

BLE 的写入回调在蓝牙栈线程上下文运行，不能在回调里做耗时/可能阻塞的操作（例如 SPI 传输），否则可能导致：

BLE 卡顿、断连

系统调度异常

数据不稳定

因此设计为：

写入回调只做：校验 + 存命令 + k_work_submit()

真正 SPI 传输在 work handler 里执行

3. 关键硬件点：为什么以前 SPI 不通、后来通了？
3.1 SD 底座引脚不是“直接裸 IO”

SD/TF 底座对应的信号线往往经过：

电平转换器

负载开关（Load Switch）

供电域（V_SD）

如果 V_SD 未使能（Load Switch 未打开），外侧线路会出现典型现象：

主机 SPI 调用 ret=0（驱动层面觉得传完了），但实际从机收不到

主机读回可能出现全 00 或随机

从机侧持续 timeout

3.2 最重要的修复：固定使能 SD 域供电（LS_SD）

最终稳定工作的做法：

在 esp32_link_init() 内部强制调用 esp32_link_set_power(true)

esp32_link_set_power(true) 会：

保持 LS_3V3 打开

打开 LS_SD，确保 V_SD 有电

延时一小段时间让电平转换稳定

（可选）读回引脚状态并打印日志，便于确认没有被其它模块拉低

经验总结：
“SPI 线没电 = 一切都是假的。”
必须先确认 V_SD = 3.3V（或预期电压）稳定存在。

3.3 第二个关键修复：CS 物理电平与 GPIO_ACTIVE_LOW 语义冲突

OpenEarable devicetree 中的 cs-gpios 常配置为 GPIO_ACTIVE_LOW。
如果用 gpio_pin_set_dt()，写入的“逻辑值”会被自动反相，导致你以为拉低 CS，实际拉高了（或相反），表现为：

ESP32 从机一直认为没有被选中（timeout）

nRF 主机读回全 00（MISO 被拉低或未驱动）

最终稳定做法：

CS 使用 raw 物理电平控制：gpio_pin_set_raw()

固定行为：0 = 物理拉低（选中），1 = 物理拉高（释放）
完全对齐你在 nRF5340 DK 上验证成功的写法。

4. 工程改动清单（必须备份的文件与位置）

以下路径以你当前工程结构为准（你之前报错路径显示 src/spi_esp32/ 与 src/bluetooth/gatt_services/）。

4.1 SPI 主机封装模块

目录： src/spi_esp32/

esp32_link.hpp

esp32_link.cpp

功能要点：

esp32_link_init()：初始化 SPI + CS + 强制打开 LS_SD 供电域

esp32_link_xfer()：全双工传输（建议所有上层都走这个）

CS 用 gpio_pin_set_raw() 控制物理电平

每次传输前可选 esp32_link_set_power(true) 兜底（防止被其他模块误关）

4.2 BLE 写入触发 SPI 的 GATT 服务（新增）

目录： src/bluetooth/gatt_services/

spi_cmd_service.h

spi_cmd_service.cpp （注意：必须是 .cpp，因为 include 了 C++ 头 esp32_link.hpp）

功能要点：

新增自定义 128-bit UUID Service + Characteristic

Characteristic 支持 Write / Write Without Response

写入 1 字节 0x01/0x02/0x03 → 触发 SPI 发送不同 4 字节 payload

work handler 里调用 esp32_link_init() + esp32_link_xfer()

RTT/LOG 输出发送与回包，便于验证

4.3 CMakeLists.txt 修改

确保把 spi_cmd_service.cpp 加入编译（示例）：

target_sources(app PRIVATE
  src/bluetooth/gatt_services/spi_cmd_service.cpp
  src/spi_esp32/esp32_link.cpp
)


注意：如果你用了 GLOB，也要确认 .cpp 确实被包含；否则会出现“手机找不到 service”或 link error。

4.4 main 的改动点

main 只需要在初始化阶段调用一次：

init_spi_cmd_service();

建议位置：与 init_led_service()、init_sensor_service() 同级，确保在 BLE 开始工作前完成注册/初始化。

原则：

不建议在 main 里永久保留“每秒自动发 SPI”的测试线程（除非你明确需要）。

推荐：SPI 由 BLE 指令触发，行为可控且易验证。

5. 运行与验证（你或别人按这套就能复现）
5.1 固件端（OpenEarable / nRF5340）

build profile：你当前能成功的 build_esp32 配置

烧录：通过 5340DK J-Link（官方推荐方式）

日志查看：SEGGER RTT（你已验证可用）

你应该看到的 RTT/LOG 关键字：

esp32_link_init done...

Power ON: LS_SD readback=1 ...

SPI cmd queued: 1

SPI cmd=1 ret=0 tx=11 22 33 44 ...

5.2 手机端（nRF Connect Mobile）

连接 OpenEarable

找到自定义 SPI CMD Service（UUID 以 7A2B0001-... 开头）

找到 Characteristic（UUID 7A2B0002-...）

Write 值：

01 → 发 11 22 33 44

02 → 发 22 33 44 55

03 → 发 33 44 55 66

5.3 ESP32 从机侧

SPI Slave 初始化成功

串口日志应出现：

RX from nRF: 11 22 33 44 等

如果持续 timeout：优先检查 CS/供电/线序/是否共地

6. 接线注意事项（必须写清楚，否则以后必踩坑）

因为你是“从 SD 底座引脚飞线”，你无法打开设备，只能从外部测量。

6.1 必须共地

OpenEarable GND 与 ESP32 GND 必须连接，否则可能出现：

ret=0 但读回随机

ESP32 timeout

波形测量误导

6.2 供电域与电平

SD 底座外侧电压来自 V_SD（由 LS_SD 控制）

你已验证：当 LS_SD=1 时底座引脚电压在 3V 左右并可跳变

若发现一直 0V：

优先确认 LS_SD 是否被拉高

确认 LS_3V3 总开关是否打开

确认没有其他模块把它关掉（例如 SD logging 功能）

6.3 CS 的“物理电平”原则

你的主机逻辑：CS 物理拉低 = 选中

如果用 dt 语义控制（gpio_pin_set_dt）可能会反相

因此最终实现选择 gpio_pin_set_raw 保证物理一致性

7. 常见问题与排障手册（将来你一定会用到）
Q1：nRF 端 ret=0 但 rx=00 00 00 00，ESP32 端 timeout

最常见原因顺序：

CS 没真正被拉低（active_low 反相 / 接错脚）

V_SD 没上电（LS_SD 未使能）

MISO/MOSI 接反

未共地

Q2：拔下 ESP32 时 nRF 读到 FF，插上变成 00

FF：MISO 线上拉/悬空高

00：MISO 被外侧拉低或被钳位
优先排查：供电域、电平转换、MISO 物理连接是否正确

Q3：为什么 .c 文件 include esp32_link.hpp 会报错 cstddef？

.c 用 C 编译器（-std=c99），不认识 C++ 标准头

解决：GATT service 文件改为 .cpp 或提供 C 版 API 头

8. 版本与环境信息（备份必须包含）

OpenEarable 工程版本：2.2.x（你当前使用的分支/包）

NCS：v3.0.1（日志中可见）

Zephyr：v4.0.99-ncs1-1（日志中可见）

构建工具：nRF Connect for VS Code + west + sysbuild

调试输出：SEGGER RTT（J-Link V8.xx）

9. 建议的备份方式（可选但强烈推荐）

复制整个工程目录作为快照，例如：

openearable_spi_blecmd_backup_YYYYMMDD/

在根目录放置本文档：

README_SPI_ESP32_BACKUP.md

在文档顶部记录：

接线照片/示意图

你最终使用的 SPI 引脚来源（SD 底座哪几脚）

你最终验证通过的 RTT 输出片段（10 行以内）

10. 下一步可扩展方向（未来你要做时会用到）

把 1/2/3 扩展为结构化命令（例如 opcode + payload）

增加从 ESP32 回包的解析与状态上报（BLE Notify）

把 SPI 命令与 OpenEarable 现有状态机挂钩（如按钮触发、传感器触发）

加互斥与队列（多指令排队发送）