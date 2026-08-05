/**
 * @file FrameTransmit.c
 * @brief DJI RS 命令帧发送实现
 *
 * rs_send_cmd() 先调用 Combine() 合成完整协议帧，再通过 CAN 总线发送；
 * 若调用方需要等待应答，可传入 seq_out 保存帧序列号，用于后续匹配应答帧。
 */
#include "FrameTransmit.h"

/**
 * @brief 通用命令帧发送（不等待应答）
 * @param cmd_set  命令集（如 0x0E 云台、0x0D 相机）
 * @param cmd_id   命令 ID
 * @param data     命令数据区（可为空）
 * @param data_len 命令数据长度
 * @param seq_out  可选：回填本帧序列号（2 字节），用于之后匹配应答帧
 * @return true=发送成功，false=失败
 */
bool rs_send_cmd(uint8_t cmd_set, uint8_t cmd_id, uint8_t *data, uint16_t data_len, uint8_t *seq_out)
{
    uint8_t *cmd;
    bool ret = false;

    /* 1. 合成完整协议帧（内部申请内存） */
    cmd = Combine(CmdType, cmd_set, cmd_id, data, data_len);
    if (cmd == RT_NULL)
        return false;

    /* 2. 按帧长字段（Ver/Length 低 10 位）计算总长度并发送 */
    if (send_data(cmd, (uint16_t)(cmd[1] | ((uint16_t)cmd[2] << 8))))
    {
        ret = true;
        if (seq_out != RT_NULL)
        {
            /* 3. 把帧中的序列号回填给调用方 */
            seq_out[0] = cmd[8];
            seq_out[1] = cmd[9];
        }
    }
    rt_free(cmd); /* 4. 释放合成时申请的内存 */
    return ret;
}
