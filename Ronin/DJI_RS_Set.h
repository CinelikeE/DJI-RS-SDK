/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-02     CinelikeE       the first version
 */
#ifndef _DJI_RS_SET_H_
#define _DJI_RS_SET_H_

#include "DJI_RS_SDK.h"



uint8_t position_ctrl_byte  ;  // 位置控制标志位
uint8_t speed_ctrl_byte     ;  // 速度控制标志位

uint8_t enc;

enum {
    NoEnc ,
    AES256
} DJI_RS_enc;

void Enc_Set(uint8_t Encode);


uint8_t CmdType;

enum Response{
    NoResponse   = 0,
    Response     = 1,
    MustResponse = 2
}   FrameResponse;

enum Type{
    CommandFrame  = 0,
    ResponseFrame = 1
}   FrameType;

void CmdType_Set(uint8_t Response, uint8_t Type);







#endif /* SRC_RONIN_DJI_RS_SET_H_ */
