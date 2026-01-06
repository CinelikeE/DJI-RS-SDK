/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-03     CinelikeE       the first version
 */
#ifndef SRC_RONIN_RESPONSE_H_
#define SRC_RONIN_RESPONSE_H_
#include "DJI_RS_SDK.h"

enum ReturnCode {
    EXECUTION_SUCCESSFUL = 0,
    PARSE_ERROR = 1,
    EXECUTION_FAILS = 2,
    UNDEFINED_ERROR = 0xFF
};

uint8_t Moveto_Response(uint8_t Res);


#endif /* SRC_RONIN_RESPONSE_H_ */
