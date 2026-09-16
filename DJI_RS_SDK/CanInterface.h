/**
 * @file CanInterface.h
 * @brief DJI RS SDK 唯一的平台中间接口（纯 C）
 *
 * SDK 内部只依赖下面这张"函数指针表"，不再包含任何平台的 CAN 头文件，
 * 也不需要在 SDK 里为每个平台写适配类。移植时只要在你自己工程里填一张
 * rs_can_driver_t，让几个函数去调用平台的 C API 即可：
 *
 *  - ESP-IDF  : twai_new_node_onchip / twai_node_transmit / twai_node_receive_from_isr
 *               （注意：新 TWAI 驱动只能在 on_rx_done 中断里取帧，所以回调里要
 *                 把帧放进一个 FreeRTOS 队列，receive() 再从队列取，参考 main.cpp）
 *  - RT-Thread: rt_device_find / rt_device_write / rt_device_read / rt_tick_get
 *  - STM32 HAL: HAL_CAN_AddTxMessage / HAL_CAN_GetRxMessage / HAL_GetTick
 *
 * 必须实现：send、receive、millis（缺任意一个 attach() 会返回 false）。
 * 可选实现：init、deinit、delay_ms（传 NULL 表示不需要）。
 *
 * 约定：
 *  - 经典 CAN、标准帧、最多 8 字节数据；一条协议帧的拆帧/组帧由 SDK 负责，
 *    本接口只收发单条 CAN 帧；
 *  - receive() 必须支持超时返回（不能永久阻塞），超时返回 false；
 *  - millis() 必须是单调递增的毫秒时基，用于判断应答超时。
 */
#ifndef RS_CAN_INTERFACE_H
#define RS_CAN_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 一帧经典 CAN 数据 */
typedef struct rs_can_frame
{
    uint32_t id;      /* 标准帧 ID（11bit）或扩展帧 ID（29bit） */
    uint8_t  data[8]; /* 数据区，经典 CAN 最多 8 字节 */
    uint8_t  len;     /* 有效字节数 0 ~ 8 */
    bool     extended;/* true = 29bit 扩展帧 */
    bool     remote;  /* true = 远程帧（DJI RS 用不到） */
} rs_can_frame_t;

/**
 * @brief 平台 CAN 驱动接口（中间接口）
 *
 * @note ctx 是给实现使用的上下文指针（例如 TWAI 节点句柄、rt_device 指针、
 *       CAN_HandleTypeDef 指针），SDK 不关心它的内容，只会原样回传。
 */
typedef struct rs_can_driver
{
    /** @brief 初始化 CAN（可选）：返回 true 表示成功 */
    bool (*init)(void *ctx);

    /** @brief 关闭 CAN（可选） */
    void (*deinit)(void *ctx);

    /** @brief 发送一帧（必须实现）：timeout_ms = 0 表示不等待 */
    bool (*send)(void *ctx, const rs_can_frame_t *frame, uint32_t timeout_ms);

    /** @brief 接收一帧（必须实现）：最多等 timeout_ms，超时返回 false */
    bool (*receive)(void *ctx, rs_can_frame_t *frame, uint32_t timeout_ms);

    /** @brief 毫秒时基（必须实现）：单调递增即可，允许溢出回绕 */
    uint32_t (*millis)(void *ctx);

    /** @brief 毫秒延时（可选） */
    void (*delay_ms)(void *ctx, uint32_t ms);

    /** @brief 实现自己的上下文指针，SDK 原样回传 */
    void *ctx;
} rs_can_driver_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RS_CAN_INTERFACE_H */
