# DJI_RS_SDK（C++ 版）

把 [CinelikeE/DJI-RS-SDK](https://github.com/CinelikeE/DJI-RS-SDK)（RT-Thread + C 版）
改写为 C++，并按 `DJI_RS_SDK.h` 中定义的类/函数名组织接口。

## 1. 目录结构

| 文件 | 说明 | 对应的原 C 文件 |
| --- | --- | --- |
| `DJI_RS_SDK.h / .cpp` | 对外 API：`RoninS` 及 `Set` / `Move` / `Get` / `FocusMotor` / `Camera` | `DJI_RS_SDK.c`、`DJI_RS_Set.c` |
| `Combine.h / .cpp` | 协议帧合成（帧头 + CRC16 + 数据段 + CRC32） | `CmdCombine.c`、`FrameTransmit.c` |
| `CmdParse.h / .cpp` | CAN 组帧状态机、CRC 校验、帧解析、参数推送解析 | `CmdParse.c` |
| `Response.h / .cpp` | 命令发送、应答匹配与超时等待 | `Response.c`、`FrameTransmit.c` |
| `CanInterface.h` | **唯一的平台中间接口**（纯 C 函数表 `rs_can_driver_t`），换平台只实现它 | `Handle.h` / `Handle.c` 的抽象部分 |
| `custom_crc16.c/.h`、`custom_crc32.c/.h` | CRC 校验，**从原仓库原样复制，未做任何修改** | 同名文件 |

## 2. 三个设计要点

1. **不使用 new / malloc**
   发送缓冲、接收缓冲、组帧缓冲、应答缓冲全部是 `RoninS` 的成员数组；
   平台侧需要缓存时也用静态存储（例程里 TWAI 的接收队列就是 `xQueueCreateStatic()`）。
   `RoninS` 对象约 1.1KB，建议定义为全局变量或静态变量。

2. **平台无关的 CAN 接口**
   SDK 只通过 `CanInterface.h` 里的 `rs_can_driver_t`（一组 C 函数指针：
   `init/deinit/send/receive/millis/delay_ms` + `ctx`）收发数据，不包含任何平台的 CAN 头文件。
   平台侧用 C 写几个函数，把 `twai.c` / `rt_device` / HAL 包一层就行。

3. **不依赖操作系统**
   SDK 内部不创建线程、不使用消息队列；接收由调用方驱动：
   - `connect()` / `Move::Joint()` 等需要应答的接口，会在内部边收边等；
   - 空闲时想处理参数推送帧（`0x0E/0x08`、`0x0E/0x10`），在任务里周期调用
     `ronin.poll(timeout_ms)`，或注册 `setPushCallback()` 后由 `poll()` 触发回调。

## 3. ESP32（ESP-IDF）使用示例

可编译的完整例子见 [main.cpp](../main.cpp)：上半部分是平台实现（直接调用 `esp_twai` 的 C API），
下半部分是 SDK 用法。核心就是先填好那张驱动表：

```c
/* ---- 1. 平台侧：用 twai.c 的 C API 实现这几个函数 ---- */
static bool     canInit(void *ctx);      /* twai_new_node_onchip + 注册中断 + twai_node_enable */
static void     canDeinit(void *ctx);    /* twai_node_disable + twai_node_delete */
static bool     canSend(void *ctx, const rs_can_frame_t *f, uint32_t t); /* transmit + wait_all_done */
static bool     canReceive(void *ctx, rs_can_frame_t *f, uint32_t t);    /* 从 FreeRTOS 队列取帧 */
static uint32_t canMillis(void *ctx);    /* esp_timer_get_time() / 1000 */

/* ---- 2. 填成 SDK 认的唯一中间接口 ---- */
static const rs_can_driver_t g_canDriver = {
    canInit, canDeinit, canSend, canReceive, canMillis, canDelay, &g_can
};

/* ---- 3. SDK 用法 ---- */
static RoninS ronin;                     /* 约 1.1KB，建议静态 */

extern "C" void app_main(void)
{
    if (!ronin.attach(g_canDriver))      /* 绑定驱动，内部会调用 canInit() */
        return;

    ronin.init();                        /* 协议配置：绝对位置、三轴有效、不加密、必须应答 */
    if (ronin.connect())                 /* 查询模块版本，确认链路 */
        ronin.move.Joint(120, 0, 30, 1); /* Yaw 120°, Pitch 0°, Roll 30°, 1s（等应答） */

    while (1) {
        ronin.poll(1000);                /* 空闲时收参数推送帧 */
    }
}
```

要点：

- **新 TWAI 驱动只能在 `on_rx_done` 中断里取帧**（`twai_node_receive_from_isr`），
  所以例程在中断里把帧投进静态 FreeRTOS 队列，`canReceive()` 再从队列取；
  SDK 拿到的是完整的 8 字节 CAN 帧，拆帧/组帧/CRC 都由 SDK 负责；
- **发送缓冲要等发送结束**：`twai_node_transmit()` 只保存帧数据指针，
  例程先把数据拷进上下文缓冲，并在 `twai_node_transmit_wait_all_done()` 之后才返回；
- 回环自测（不接云台）时把 `main.cpp` 里的 `CAN_LOOPBACK_DEBUG` 改成 `true`，
  TX/RX 复用同一个 GPIO，并开启 `enable_loopback` / `enable_self_test`；
- 一个 TWAI 控制器只能由一个地方初始化，不要和别的代码同时建节点。

## 4. 移植到其它平台（只实现一张 C 函数表）

SDK 内部不认识任何平台的 API，只调用 `rs_can_driver_t` 里的函数指针，
所以平台相关代码全部卸到应用侧，而且可以纯 C 写。以 RT-Thread 为例（对应原 `Handle.c`）：

```c
static bool can_init(void *ctx)     { /* rt_device_find + open + 配置滤波器 */ return true; }
static void can_deinit(void *ctx)   { /* rt_device_close */ }

static bool can_send(void *ctx, const rs_can_frame_t *f, uint32_t t)
{
    struct rt_can_msg msg = {0};
    msg.id = 0x223;                 /* 本机发送 ID */
    msg.ide = RT_CAN_STDID;         /* 标准帧 */
    msg.rtr = RT_CAN_DTR;           /* 数据帧 */
    msg.len = f->len;
    for (uint8_t i = 0; i < f->len; ++i) msg.data[i] = f->data[i];
    return rt_device_write((rt_device_t)ctx, 0, &msg, sizeof(msg)) != 0;
}

static bool can_receive(void *ctx, rs_can_frame_t *f, uint32_t t)
{
    /* 用信号量/邮箱等到数据后 rt_device_read，超时返回 false */
}

static uint32_t can_millis(void *ctx) { return rt_tick_get() * 1000 / RT_TICK_PER_SECOND; }
static void can_delay(void *ctx, uint32_t ms) { rt_thread_mdelay(ms); }

static const rs_can_driver_t g_canDriver = {
    can_init, can_deinit, can_send, can_receive, can_millis, can_delay, can_dev
};
```

要点：

- **必须实现**：`send`、`receive`、`millis`；`init` / `deinit` / `delay_ms` 可以传 `NULL`；
- DJI RS 使用 **经典 CAN、500kbit/s、标准帧**；本机发送 ID = `0x223`，接收 ID = `0x222`；
- 一帧 CAN 最多 8 字节，一条协议帧（18 字节以上）的**拆帧、组帧、CRC 校验都在 SDK 内部**完成，
  驱动只需要收发单帧；
- `receive()` 必须支持超时返回（不能永久阻塞）；`millis()` 必须单调递增（允许溢出回绕）；
- 驱动表会被 `attach()` 按值拷贝，但 `ctx` 指向的资源要保证比 `RoninS` 活得久。

## 5. 与原始 C 版的差异

| 项目 | 原 C 版 | 本 C++ 版 |
| --- | --- | --- |
| 内存 | `rt_malloc` / `rt_free` 组帧 | 调用方/成员固定缓冲，无动态内存 |
| 线程 | 接收线程 + 消息队列 + 解析线程 | 无（由业务任务调用 `poll()` / 等待应答时内部接收） |
| 全局变量 | `position_ctrl_byte`、`CmdType` 等全局量 | `RoninS` 成员，支持多实例（多路 CAN） |
| 接口 | 全局函数 `move_to()`、`get_gimbal_angle()` 等 | `RoninS` 类及嵌套类 `set` / `move` / `get` / `focusMotor` / `camera` |
| CRC | `custom_crc16.c/h`、`custom_crc32.c/h` | 原样复制，未改动 |

另外补充了两处原 C 版有、但头文件里没列出的接口：

- `Set::user_param()`：协议 2.3.4.13 设置用户参数（TLV）；
- `Camera::action()` / `Camera::get_status()`：协议 2.3.5 相机控制；
- `FocusMotor::calibrate()` / `get_position()` 由 `void` 改为返回 `uint8_t`
  （返回云台返回码，调用方可忽略返回值，行为兼容）。

## 6. 注意事项

- **单命令流**：应答匹配基于"序列号 + 命令集 + 命令 ID"，同一时刻只支持一个等待者，
  多任务并发调用请自行加锁（与原实现一致）。
- **推送回调**：回调里的 `RsMessage` 指向内部缓冲，需要保留数据请立即拷贝；
  回调在调用 `poll()` / 等待应答的任务上下文执行，不要在里面长时间阻塞。
- **角度单位**：角度 0.1°、速度 0.1°/s、运动时间 0.1s；`Set::limit_angle()` 单位 1°。
- **回环自测**：命令帧不会被误当成应答（解析时要求 `CmdType[5] == 1`）。
- 不同固件版本对取值范围的限制可能不同，最终以云台返回码与协议文档为准。

## 7. 关于"用 C 写业务代码"

- **平台侧本来就是纯 C**：`CanInterface.h` 可以直接被 `.c` 文件 include，
  驱动表用 C 填好后交给 `ronin.attach()`（见第 3、4 节），不需要任何 C++ 语法；
- **SDK 对外 API 是 C++**：`RoninS` 是类，还带嵌套类 `set/move/get` 和引用成员，
  所以 `.c` 文件不能直接 `#include "DJI_RS_SDK.h"`。如果业务代码必须是 C，推荐把
  "调用 SDK"的那一层放在 `.cpp` 文件里（C 代码只调用你自己暴露的函数），
  或者在 C++ 侧包一层 `extern "C"`，例如：

```cpp
extern "C" void rs_move_joint(float yaw, float pitch, float roll, float time_s)
{
    ronin.move.Joint(yaw, pitch, roll, time_s);
}
```
