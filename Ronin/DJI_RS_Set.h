/**
 * @file DJI_RS_Set.h
 * @brief DJI RS SDK 协议配置项（头文件）
 *
 * 定义帧头 CmdType 字节（响应类型 + 帧类型）、加密类型等全局配置，
 * 并提供对应的设置接口。这些配置项在 dji_rs_sdk_init() 中初始化。
 */
#ifndef _DJI_RS_SET_H_
#define _DJI_RS_SET_H_

#include <stdint.h>

extern uint8_t position_ctrl_byte;   /* 位置控制标志位 */
extern uint8_t speed_ctrl_byte;      /* 速度控制标志位 */
extern uint8_t enc;                  /* 加密类型及补充字节长度 */
extern uint8_t CmdType;              /* 指令类型字节 */

/* 加密类型 */
typedef enum {
    RS_ENC_NONE,
    RS_ENC_AES256
} RS_EncType;

/* 应答类型（CmdType[4:0]） */
typedef enum {
    RS_RESP_NONE = 0,
    RS_RESP_OPTIONAL = 1,
    RS_RESP_MUST = 2
} RS_ResponseType;

/* 帧类型（CmdType[5]） */
typedef enum {
    RS_FRAME_COMMAND = 0,
    RS_FRAME_RESPONSE = 1
} RS_FrameType;

/* 设置加密类型 */
void Enc_Set(RS_EncType encode);
/* 设置帧头 CmdType（应答类型 + 帧类型） */
void CmdType_Set(RS_ResponseType response, RS_FrameType type);

#endif /* _DJI_RS_SET_H_ */
