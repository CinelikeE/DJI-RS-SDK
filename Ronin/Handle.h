/**
 * @file Handle.h
 * @brief CAN 总线收发模块（头文件）
 *
 * 负责 CAN 设备的初始化、接收线程与消息队列，以及把任意长度的
 * 协议帧拆分为多个 8 字节 CAN 帧发送的 send_data()。
 *
 * 协议约定：PC 侧（本机）CAN Tx = 0x223，Rx = 0x222。
 */
#ifndef HANDLE_H
#define HANDLE_H

#include <stdint.h>
#include "stdbool.h"
#include "rtthread.h"
#include "rtdevice.h"
#include "board.h"
#include "drivers/can.h"

/* 文档 3.1：PC 侧（本机）CAN Tx = 0x223，Rx = 0x222 */
#define RS_CAN_TX_ID 0x223
#define RS_CAN_RX_ID 0x222

/* CAN 消息结构体，用于消息队列传递 */
typedef struct {
    uint32_t id;      /* CAN 帧 ID（0x222 应答 / 0x223 发送） */
    uint8_t data[8];  /* CAN 数据（标准帧最多 8 字节） */
    uint8_t len;      /* 有效数据长度 */
} CanMsg;

extern rt_mq_t can_rx_mq; /* CAN 接收消息队列，由解析线程消费 */

int rt_Can_init(void);                            /* 初始化 CAN 设备并启动接收线程 */
bool send_data(uint8_t *data, uint16_t data_len); /* 把协议帧按 8 字节拆帧发送 */

#endif /* HANDLE_H */
