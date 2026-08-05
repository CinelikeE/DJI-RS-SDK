/**
 * @file CmdParse.h
 * @brief DJI RS 协议帧解析与推送消息定义（头文件）
 *
 * 提供：
 *  - RS_Msg：应答/推送帧的解析结果结构
 *  - FrameBuffer：帧接收缓冲及解析状态机
 *  - RS_PushParam：2.3.4.9 手持云台参数推送结构
 *  - Parse()：解析线程入口，负责把 CAN 消息拼接成完整协议帧
 */
#ifndef CMDPARSE_H
#define CMDPARSE_H

#include "rtthread.h"
#include "CmdCombine.h"
#include "custom_crc16.h"
#include "custom_crc32.h"
#include "Handle.h"
#include "DJI_RS_Set.h"

/* 参数推送 / 校准状态推送命令 ID */
#define RS_CMD_PUSH_PARAM 0x08
#define RS_CMD_PUSH_CALIB 0x10

/* 通用应答/推送消息：由解析线程从 can_rx_mq 取出完整帧后填入 */
typedef struct {
    uint8_t seq[2];       /* 序列号，与命令帧一致 */
    uint8_t frame_type;   /* CmdType[5]：0 命令帧 / 1 应答帧 */
    uint8_t cmd_set;      /* 命令集 */
    uint8_t cmd_id;       /* 命令码 */
    uint8_t data[RS_MAX_FRAME_LEN]; /* 数据段中 CmdData 部分（应答帧含返回码） */
    uint16_t len;         /* data 有效长度 */
} RS_Msg;

/* 应答消息队列句柄 */
extern rt_mq_t rs_res_mq;

/* 帧解析状态 */
typedef enum {
    FRAME_IDLE,
    FRAME_RECEIVING,
    FRAME_COMPLETE
} FrameParseState;

/* 帧缓冲区 */
typedef struct {
    uint8_t buffer[RS_MAX_FRAME_LEN]; /* 最大帧长 */
    uint16_t len;         /* 当前已接收长度 */
    uint16_t len_max;     /* 完整帧长度 */
    FrameParseState state;
} FrameBuffer;

/* 2.3.4.9 手持云台参数推送解析结果 */
typedef struct {
    uint8_t valid_flag;               /* [0] 角度信息有效 [1] 限位信息有效 [2] 电机力度信息有效 */
    int16_t yaw_angle;
    int16_t roll_angle;
    int16_t pitch_angle;
    int16_t yaw_joint_angle;
    int16_t roll_joint_angle;
    int16_t pitch_joint_angle;
    uint8_t pitch_max;
    uint8_t pitch_min;
    uint8_t yaw_max;
    uint8_t yaw_min;
    uint8_t roll_max;
    uint8_t roll_min;
    uint8_t pitch_stiffness;
    uint8_t yaw_stiffness;
    uint8_t roll_stiffness;
} RS_PushParam;

/* 推送回调（0x0E/0x08 参数推送、0x0E/0x10 校准状态推送）。
 * 注意：msg 指向解析线程内部静态缓冲区，回调内请立即拷贝所需数据。 */
typedef void (*rs_push_callback_t)(RS_Msg *msg);
void rs_set_push_callback(rs_push_callback_t cb);

/* 将 0x0E/0x08 推送消息解析为结构体 */
void rs_parse_push_param(const RS_Msg *msg, RS_PushParam *out);

void Parse(void *parameter);

#endif /* CMDPARSE_H */
