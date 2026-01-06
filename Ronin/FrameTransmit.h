/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-03     CinelikeE    the first version
 */
#ifndef _FRAMETRANSMIT_H_
#define _FRAMETRANSMIT_H_
#include "DJI_RS_SDK.h"
#include "stdbool.h"
#include "CmdCombine.h"
#include "Handle.h"
#include "DJI_RS_Set.h"



bool moveto(uint8_t *seq ,float yaw_angle, float roll_angle, float pitch_angle, float time_s);



#endif /* SRC_RONIN_FRAMETRANSMIT_H_ */
