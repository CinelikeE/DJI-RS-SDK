#ifndef CMDPARSE_H
#define CMDPARSE_H
#include "DJI_RS_SDK.h"
#include "custom_crc16.h"
#include "custom_crc32.h"
#include "handle.h"

// 帧解析状态枚举
typedef enum {
    FRAME_IDLE,
    FRAME_SOF_FOUND,
    FRAME_RECEIVING,
    FRAME_COMPLETE
} FrameParseState;

// 帧缓冲区结构体
typedef struct {
    uint8_t buffer[256];  // 最大帧长
    uint16_t len;         // 当前已接收长度
    FrameParseState state; // 解析状态
} FrameBuffer;

void Parse();






#endif //CMDPARSE_H
