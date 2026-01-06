/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-03     CinelikeE       the first version
 */
#include "FrameTransmit.h"


bool moveto(uint8_t *seq ,float yaw_angle, float roll_angle, float pitch_angle, float time_s){

    bool ret = false; // 初始化返回值，避免随机值

    int16_t yaw = (int16_t)(yaw_angle * 10);
    int16_t roll = (int16_t)(roll_angle * 10);
    int16_t pitch = (int16_t)(pitch_angle * 10);
    uint8_t time = (uint8_t)(time_s * 10);

    if (!(yaw >= -1800 && yaw <= 1800 &&
          roll >= -300 && roll <= 300 &&  // 横滚限制更严格
          pitch >= -560 && pitch <= 1460 &&  // 俯仰范围
          time >= 1)) {  // 最小时间0.1s
        rt_kprintf("Error!! FrameTransmit.c Function: moveto overflow!\n");
        return RT_ERROR;
    }

    // 数据载荷 (小端模式)
    uint8_t data_payload[] = {
        yaw & 0xFF,        (yaw >> 8) & 0xFF,   // Yaw：低字节→高字节（小端）
        roll & 0xFF,       (roll >> 8) & 0xFF,  // Roll：低字节→高字节（小端）
        pitch & 0xFF,      (pitch >> 8) & 0xFF, // Pitch：低字节→高字节（小端）
        position_ctrl_byte,                     // 控制标志位
        time                                    // 执行时间
    };

    uint8_t *cmd = Combine(CmdType, 0x0E, 0x00, data_payload, sizeof(data_payload));
    if (!cmd) return RT_FALSE;



    if(ret = send_data(cmd, cmd[1])){  // cmd[1]是帧长度

        if(seq != RT_NULL){
        seq[0] = cmd[8];
        seq[1] = cmd[9];
        }
        rt_free(cmd);
    }
    return ret;

}
