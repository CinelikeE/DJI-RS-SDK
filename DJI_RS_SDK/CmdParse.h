/**
 * @file CmdParse.h
 * @brief DJI RS 协议帧解析（对应原 C 版 CmdParse.c）
 *
 * 组帧状态机：把多条 8 字节 CAN 数据拼成一条完整协议帧，
 * 并依次校验 SOF/ENC/RES、CRC16、CRC32，校验通过后解析为 RsMessage。
 *
 * 与原 C 实现的区别：
 *  - 不再依赖 RT-Thread 消息队列和接收线程；由用户任务调用 feed() 驱动；
 *  - 接收缓冲、状态机全部是类成员，不做动态内存分配。
 */
#ifndef RS_CMD_PARSE_H
#define RS_CMD_PARSE_H

#include <stdint.h>
#include "Combine.h"

/* 参数推送 / 校准状态推送的命令 ID（协议 2.3.4.9 / 2.3.4.16） */
#define RS_CMD_PUSH_PARAM 0x08
#define RS_CMD_PUSH_CALIB 0x10

/* 单条消息缓存 CmdData 的最大长度（协议应答都很短，64 字节足够） */
#define RS_MSG_DATA_MAX 64

/** @brief 一条完整的协议帧（应答帧或推送帧） */
struct RsMessage
{
    uint8_t  seq[2];     /* 序列号，与命令帧一致 */
    uint8_t  frame_type; /* CmdType[5]：0 = 命令帧，1 = 应答/推送帧 */
    uint8_t  cmd_set;    /* 命令集 */
    uint8_t  cmd_id;     /* 命令 ID */
    uint16_t len;        /* data 中的有效字节数（应答帧 data[0] 为返回码） */
    uint8_t  data[RS_MSG_DATA_MAX];
};

/** @brief 协议 2.3.4.9 手持云台参数推送解析结果 */
struct RsPushParam
{
    uint8_t valid_flag; /* [0] 角度有效 [1] 限位有效 [2] 电机力度有效 */
    int16_t yaw_angle;  /* 姿态角，单位 0.1° */
    int16_t roll_angle;
    int16_t pitch_angle;
    int16_t yaw_joint_angle; /* 关节角，单位 0.1° */
    int16_t roll_joint_angle;
    int16_t pitch_joint_angle;
    uint8_t pitch_max; /* 限位角度，单位 1° */
    uint8_t pitch_min;
    uint8_t yaw_max;
    uint8_t yaw_min;
    uint8_t roll_max;
    uint8_t roll_min;
    uint8_t pitch_stiffness; /* 电机力度 0 ~ 100 */
    uint8_t yaw_stiffness;
    uint8_t roll_stiffness;
};

class CmdParse
{
public:
    CmdParse();

    /** @brief 清空接收缓冲与状态机（超时或校验失败时调用） */
    void reset();

    /**
     * @brief 送入一帧 CAN 数据
     * @param data CAN 数据指针（按 8 字节缓冲区处理，不足部分视为 0）
     * @param len  本帧有效字节数（1 ~ 8）
     * @param enc  本机配置的加密类型，用于校验帧头 ENC 字段
     * @param out  组成完整帧且 CRC 校验通过时，输出解析结果
     * @return true = 本次送入后组成了完整协议帧
     */
    bool feed(const uint8_t *data, uint8_t len, uint8_t enc, RsMessage &out);

    /** @brief 把 0x0E/0x08 参数推送消息解析为结构体 */
    static void parsePushParam(const RsMessage &msg, RsPushParam &out);

private:
    enum ParseState
    {
        STATE_IDLE = 0, /* 等待帧头 */
        STATE_RECEIVING, /* 正在接收 */
        STATE_COMPLETE   /* 完整帧就绪（内部使用） */
    };

    /** @brief 从接收缓冲中提取字段，生成 RsMessage */
    void makeMessage(RsMessage &out) const;

    uint8_t    m_buffer[RS_MAX_FRAME_LEN]; /* 协议帧接收缓冲 */
    uint16_t   m_len;                      /* 当前已接收长度 */
    uint16_t   m_lenMax;                   /* 本帧完整长度 */
    ParseState m_state;
};

#endif /* RS_CMD_PARSE_H */
