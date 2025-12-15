#ifndef HANDLE_H
#define HANDLE_H
#include "DJI_RS_SDK.h"
#include "stdint.h"

#include "rtthread.h"
#include "rtdevice.h"
#include "board.h"
#include "drivers/can.h"



// 定义CAN消息结构体，用于消息队列传递
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
} CanMsg;

// 声明消息队列句柄
extern rt_mq_t can_rx_mq;

int rt_Can_init();
uint32_t send_data(uint8_t *data, uint8_t data_len);




int CanTX_Init();
uint32_t send_data(uint8_t *data, uint8_t data_len);

#endif //HANDLE_H
