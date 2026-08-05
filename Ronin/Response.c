/**
 * @file Response.c
 * @brief 应答帧等待与“发送并等待应答”接口实现
 *
 * rs_wait_response() 从 rs_res_mq 队列中取出应答帧，并按
 * 序列号/命令集/命令 ID 匹配；不匹配的帧会被丢弃。
 * 注意：本实现假设同一时刻只有一个等待者（单命令流），
 * 否则不匹配的应答帧可能被错误丢弃。
 */
#include "Response.h"
#include "DJI_RS_SDK.h"

/* @brief 等待指定 SEQ/命令的应答帧
 * @param seq        本机发送命令帧时的序列号
 * @param cmd_set    命令集
 * @param cmd_id     命令 ID
 * @param out        输出缓冲（应答帧 CmdData，out[0] 为返回码）
 * @param out_max    out 缓冲大小
 * @param out_len    实际写入长度
 * @param timeout_ms 超时时间（毫秒）
 * @return 云台返回码；超时返回 RS_TIMEOUT(0x03)，错误返回 RS_ERROR(0x04) */
uint8_t rs_wait_response(uint8_t seq[2], uint8_t cmd_set, uint8_t cmd_id,
                         uint8_t *out, uint8_t out_max, uint8_t *out_len,
                         uint32_t timeout_ms)
{
    /* 注意：本函数假设只有一个等待者（单命令流）。否则不匹配应答帧会被丢弃。推送帧已不进入本队列。 */
    RS_Msg msg;
    rt_err_t res;
    rt_tick_t start_tick = rt_tick_get();
    rt_tick_t timeout_tick = RT_TICK_PER_SECOND * timeout_ms / 1000;

    /* 在超时时间内循环等待应答队列 */
    while (1)
    {
        /* 先检查总超时，超时则返回 RS_TIMEOUT */
        if (rt_tick_get() - start_tick > timeout_tick)
            return RS_TIMEOUT;

        /* 每次从队列取一条消息，100ms 超时以便周期检查总超时 */
        res = rt_mq_recv(rs_res_mq, &msg, sizeof(RS_Msg), 100);
        /* 只匹配应答帧（CmdType[5]=1），避免自环/推送帧误匹配；
         * 同时匹配序列号、命令集与命令 ID */
        if (res >= 0)
        {
            if (msg.frame_type == 1 &&
                msg.seq[0] == seq[0] && msg.seq[1] == seq[1] &&
                msg.cmd_set == cmd_set && msg.cmd_id == cmd_id)
            {
                /* 拷贝应答数据（长度受调用方缓冲限制） */
                uint16_t n = (msg.len > out_max) ? out_max : msg.len;
                if (out != RT_NULL && n > 0)
                    rt_memcpy(out, msg.data, n);
                if (out_len != RT_NULL)
                    *out_len = n;
                /* 应答帧首字节为云台返回码 */
                return (msg.len > 0) ? msg.data[0] : RS_ERROR;
            }
        }
        else if (res == -RT_ETIMEOUT)
        {
            /* 队列暂时为空，继续循环等待 */
            continue;
        }
        else
        {
            /* 队列异常，直接返回错误 */
            return RS_ERROR;
        }
    }
}

/* @brief 发送命令帧并等待应答
 * @return 云台返回码；发送失败返回 RS_ERROR(0x04) */
uint8_t rs_send_and_wait(uint8_t cmd_set, uint8_t cmd_id,
                         uint8_t *payload, uint8_t payload_len,
                         uint8_t *out, uint8_t out_max, uint8_t *out_len,
                         uint32_t timeout_ms)
{
    uint8_t seq[2] = {0, 0};

    /* 先发送命令帧，并通过 seq 保存本帧序列号 */
    if (!rs_send_cmd(cmd_set, cmd_id, payload, payload_len, seq))
        return RS_ERROR;

    /* 再用该序列号等待匹配的应答帧 */
    return rs_wait_response(seq, cmd_set, cmd_id, out, out_max, out_len, timeout_ms);
}
