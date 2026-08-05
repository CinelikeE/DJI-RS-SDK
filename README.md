# DJI-RS-SDK For RT-Thread

> 用 RT-Thread 控制大疆稳定器（DJI RS 2 / RS 3 Pro 等支持 DJI RS SDK 的机型）的 C 实现。  
> 开发板：RT-Thread 官方 HMI-Board（瑞萨 Renesas RA6M3），IDE：RT-Thread Studio。

## 简介 Overview

在 RT-Thread OS 上控制大疆 RS稳定器的 DJI RS SDK 源代码（初代如影S不行），用 C 重新实现了参考项目 [ConstantRobotics/DJIR_SDK](https://github.com/ConstantRobotics/DJIR_SDK)（C++ 实现）。

当前代码已经具备完整的收发解析链路：

- CAN 收发：发送 0x223、接收 0x222，长帧自动按 8 字节拆分/拼接；
- 协议帧处理：组帧、CRC16/CRC32 校验、状态机解析；
- 应答机制：按序列号 + 命令集 + 命令 ID 匹配应答，支持超时；
- 推送回调：参数推送（2.3.4.9）与校准状态推送（2.3.4.17）；
- 业务封装：云台命令集（2.3.4）与相机命令集（2.3.5）常用命令；
- 示例线程：Yaw 轴 ±120° 往返运动并打印姿态角。

## 版本说明 Version

- **Ver2**：用deepseek v4 flash完善功能，已成功运行控制 RS2。部分功能待验证（ps吃了将近5000万token才花了3块钱，梁圣的恩情还不完系列）

## 吐槽 Let's say F-word


1. **RA6M3 的 CAN 一言难尽**

   RTT 里打开硬件 CAN filter 时 CAN 总线无法打开直接程序卡死；之前用 STM32 的时候是正常的。个人体验是 RTT 的兼容性对 Cortex-M 和 RISC-V 似乎都还行，但实际用下来还是 STM32 最省心，同样是 Cortex-M4 的 RA6M3 在 CAN 总线上体验就没那么好。所以如果 CAN 起不来，建议优先换 STM32 试。（后来RA6M3相关issue似乎有提这玩意根本没做硬件can filter，但芯片官方文档上题到了硬件can filter）

2. **为什么代码里全是 malloc / free**

   RTT 接管了内存管理，协议组帧又用了不少指针，所以直接用了 `rt_malloc()` / `rt_free()`。凑合能用，懒得改（能跑起来就不要动他系列）。

3. **为什么还留着 HAL 版本（Ver0）**

   - 写 HAL 版本的时候发现 CAN 邮箱一次只能缓冲约 3 帧数据，需要延时等待邮箱清空；
   - 后来 RTT 进一步封装了 MCU 外设，跨平台/跨架构确实方便，但 STM32 上 HAL 的 CAN 怎么弄我有点忘了，懒得改×2；
   - 留着当参考，想做 HAL 开发的话可以看，但具体实现以更新版本为准。

4. **有功能可能有bug**

   框架为22年开始手搓，拖到今年1月初步做到了控制，其余功能基本上由AI完善，接下来有空会逐渐开始进行测试，欢迎提issue。

## 快速使用 Instructions for Use

1. 打开 RTT 的 CAN 设备（使能 `BSP_USING_CAN`，设备名为 `can0`）；
2. 在需要的地方 `#include "DJI_RS_SDK.h"`；
3. 创建一个线程，然后该让他干啥就干啥。


业务代码示例：

```c
#include "DJI_RS_SDK.h"

void user_task(void *param)
{
    dji_rs_sdk_init();          /* 设置控制标志、加密与 CmdType */
    if (!rs_connect())          /* 查询模块版本，确认链路 */
        return;

    move_to(90.0f, 0.0f, 0.0f, 2.0f);   /* 2 秒内转到 Yaw=90° */
    rt_thread_mdelay(3000);

    int16_t yaw = 0, roll = 0, pitch = 0;
    if (get_gimbal_angle(GIMBAL_ATTITUDE_ANGLE, &yaw, &roll, &pitch)
            == EXECUTION_SUCCESSFUL)
    {
        /* 返回值单位为 0.1°，例如 yaw=900 表示 90.0° */
        rt_kprintf("yaw=%d roll=%d pitch=%d (0.1 deg)\n", yaw, roll, pitch);
    }
}
```

## 目录结构 File Structure

| 文件 | 说明 |
| --- | --- |
| `DJI_RS_SDK.h/.c` | 对外业务 API：云台、相机命令封装（协议 2.3.4 / 2.3.5） |
| `DJI_RS_Set.h/.c` | 协议配置：CmdType 字节、加密类型、位置/速度控制标志 |
| `CmdCombine.h/.c` | 协议帧合成：组帧、序列号生成 |
| `CmdParse.h/.c` | 帧解析：CAN 组帧状态机、应答队列、推送回调、解析线程 |
| `FrameTransmit.h/.c` | 命令帧发送：`rs_send_cmd()`（位置控制发送封装见 `DJI_RS_SDK.c`） |
| `Handle.h/.c` | CAN 驱动层：设备初始化、接收线程、消息队列、拆帧发送 |
| `Response.h/.c` | 应答等待：`rs_wait_response()`、`rs_send_and_wait()` |
| `custom_crc16.h/.c` | CRC16 校验（pycrc 生成，查表法） |
| `custom_crc32.h/.c` | CRC32 校验（pycrc 生成，查表法） |
| `Ronin.c` | 应用示例：位置控制演示线程 |
| `SConscript` | RT-Thread 构建脚本（依赖 `BSP_USING_CAN`） |

## 协议约定速查 Protocol Quick Reference

- CAN ID：本机发送 `0x223`，接收 `0x222`（文档 3.1）；
- 帧格式：`SOF(0xAA) | Ver/Length | CmdType | ENC | RES | SEQ | CRC16 | DATA | CRC32`；
- 单位：角度 0.1°、速度 0.1°/s、运动时间 0.1s；
- 2.3.4.1 位置控制角度范围：yaw -180.0° ~ +180.0°、roll -30.0° ~ +30.0°、pitch -56.0° ~ +146.0°。
  （`rs_send_move_to()` 目前统一按 ±180.0° 做粗校验，没有严格区分各轴；要严格校验自己按上面的范围加。）

## 注意事项 Notes

- **单命令流**：应答等待基于全局队列 `rs_res_mq`，同一时刻只支持一个等待者，多线程并发调用请自己加锁；
- **推送回调**：`msg` 指向解析线程内部静态缓冲，回调里需要保留数据请立即拷贝；
- **返回值**：返回 `uint8_t` 的接口返回云台返回码（`0x00` 成功）或本地错误码 `RS_TIMEOUT(0x03)` / `RS_ERROR(0x04)`；
- **回环自测**：定义宏 `DJI_RS_SDK_CAN_LOOPBACK` 可把 CAN 配置为回环模式；
- **固件差异**：不同机型/固件版本支持的取值范围可能不同，最终以云台返回码和 2.3.4.4 限位设置为准。

## 参考资料 References

- [ConstantRobotics/DJIR_SDK](https://github.com/ConstantRobotics/DJIR_SDK) —— 官方 C++ 参考实现
- 大疆《DJI R SDK 协议及使用接口》（CHS v2.5）
