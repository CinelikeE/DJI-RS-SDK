#ifndef CMDPARSE_H
#define CMDPARSE_H
#include "DJI_RS_SDK.h"
#include "custom_crc16.h"
#include "custom_crc32.h"
#include "handle.h"
#include "Response.h"
#include "DJI_RS_Set.h"


// 定义CAN消息结构体，用于消息队列传递
typedef struct {
    uint8_t seq[2];
    uint8_t data[50];

} RS_Msg;

// 声明消息队列句柄
extern rt_mq_t rs_res_mq;

// 帧解析状态枚举
typedef enum {
    FRAME_IDLE,
    FRAME_SOF_FOUND,
    FRAME_RECEIVING,
    FRAME_COMPLETE
} FrameParseState;

// 帧缓冲区结构体
typedef struct {
    uint8_t buffer[50];  // 最大帧长
    uint8_t len;         // 当前已接收长度
    uint8_t len_max;
    FrameParseState state; // 解析状态
} FrameBuffer;

void Parse();






#endif //CMDPARSE_H
