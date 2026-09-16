/**
 * @file Response.cpp
 * @brief 命令帧发送与应答等待实现（对应原 C 版 FrameTransmit.c + Response.c）
 *
 * 与原实现的区别：
 *  - 原版依赖 RT-Thread 接收线程 + rs_res_mq 消息队列；
 *  - C++ 版不创建线程，发送后在本函数内循环"接收 -> 组帧 -> 匹配序列号"，
 *    因此同一时刻只支持一条命令等待应答（单命令流），这与原实现一致。
 */
#include "DJI_RS_SDK.h"

bool RoninS::sendCmd(uint8_t cmd_set, uint8_t cmd_id, const uint8_t *data,
                     uint16_t data_len, uint8_t *seq_out)
{
    uint16_t frame_len;
    uint16_t offset;

    if (!isAttached())
        return false;

    /* 1. 组帧：帧头 + CRC16 + 数据段 + CRC32 */
    frame_len = m_combine.build(m_txBuffer, (uint16_t)sizeof(m_txBuffer),
                                m_cmdType, m_enc, cmd_set, cmd_id, data, data_len);
    if (frame_len == 0)
        return false;

    /* 2. 一条协议帧可能超过 8 字节，按 8 字节拆成多条 CAN 帧发送 */
    for (offset = 0; offset < frame_len;)
    {
        rs_can_frame_t frame;
        uint8_t chunk = (uint8_t)((frame_len - offset >= 8) ? 8 : (frame_len - offset));

        frame.id = RS_CAN_TX_ID;
        frame.extended = false;
        frame.remote = false;
        frame.len = chunk;
        for (uint8_t j = 0; j < chunk; ++j)
            frame.data[j] = m_txBuffer[offset + j];

        if (!m_driver.send(m_driver.ctx, &frame, m_timeoutMs))
            return false;

        offset = (uint16_t)(offset + chunk);
    }

    /* 3. 回填本帧序列号，供调用方匹配应答 */
    if (seq_out != nullptr)
    {
        seq_out[0] = m_txBuffer[8];
        seq_out[1] = m_txBuffer[9];
    }
    return true;
}

bool RoninS::serviceReceive(uint32_t timeout_ms)
{
    rs_can_frame_t frame;
    RsMessage msg;

    if (!isAttached())
        return false;

    if (!m_driver.receive(m_driver.ctx, &frame, timeout_ms))
        return false;

    /* 只关心云台应答（0x222）与自环/推送（0x223）*/
    if (frame.id != RS_CAN_RX_ID && frame.id != RS_CAN_TX_ID)
        return true;

    /* 组帧 + CRC 校验；帧不完整或校验失败时返回 false，继续等下一帧 */
    if (!m_parser.feed(frame.data, frame.len, m_enc, msg))
        return true;

    dispatchMessage(msg);
    return true;
}

bool RoninS::dispatchMessage(const RsMessage &msg)
{
    /* 参数推送（0x0E/0x08）、校准状态推送（0x0E/0x10）交给回调，不进应答缓冲 */
    if (msg.cmd_set == RS_CMD_SET_GIMBAL &&
        (msg.cmd_id == RS_CMD_PUSH_PARAM || msg.cmd_id == RS_CMD_PUSH_CALIB))
    {
        if (m_pushCb != nullptr)
            m_pushCb(msg, m_pushUser);
        return true;
    }

    /* 只缓存应答帧（CmdType[5] = 1）：回环自测时本机命令帧也会被收回，需排除 */
    if (msg.frame_type != (uint8_t)RS_FRAME_RESPONSE)
        return true;

    pushPending(msg);
    return true;
}

void RoninS::pushPending(const RsMessage &msg)
{
    if (m_pendingCount >= RS_PENDING_MAX)
    {
        /* 缓冲已满：丢弃最旧的一条，为新应答腾位置 */
        for (uint8_t i = 1; i < RS_PENDING_MAX; ++i)
            m_pending[i - 1] = m_pending[i];
        m_pendingCount = (uint8_t)(RS_PENDING_MAX - 1);
    }

    m_pending[m_pendingCount] = msg;
    m_pendingCount++;
}

bool RoninS::takePending(const uint8_t *seq, uint8_t cmd_set, uint8_t cmd_id,
                         uint8_t *out, uint8_t out_max, uint16_t *out_len,
                         uint8_t *code)
{
    for (uint8_t i = 0; i < m_pendingCount; ++i)
    {
        const RsMessage &msg = m_pending[i];

        /* 按序列号 + 命令集 + 命令 ID + 应答帧类型匹配 */
        if (msg.frame_type != (uint8_t)RS_FRAME_RESPONSE ||
            msg.seq[0] != seq[0] || msg.seq[1] != seq[1] ||
            msg.cmd_set != cmd_set || msg.cmd_id != cmd_id)
        {
            continue;
        }

        uint16_t n = (msg.len > out_max) ? out_max : msg.len;
        if (out != nullptr)
        {
            for (uint16_t j = 0; j < n; ++j)
                out[j] = msg.data[j];
        }
        if (out_len != nullptr)
            *out_len = n;
        if (code != nullptr)
            *code = (msg.len > 0) ? msg.data[0] : (uint8_t)RS_ERROR;

        /* 从缓冲中移除这条应答 */
        for (uint8_t j = (uint8_t)(i + 1); j < m_pendingCount; ++j)
            m_pending[j - 1] = m_pending[j];
        m_pendingCount--;
        return true;
    }

    return false;
}

uint8_t RoninS::waitResponse(const uint8_t *seq, uint8_t cmd_set, uint8_t cmd_id,
                             uint8_t *out, uint8_t out_max, uint16_t *out_len,
                             uint32_t timeout_ms)
{
    uint32_t start;
    uint8_t code = RS_ERROR;

    if (!isAttached())
        return RS_ERROR;

    start = m_driver.millis(m_driver.ctx);

    for (;;)
    {
        /* 1. 先检查已收到的应答里有没有匹配的 */
        if (takePending(seq, cmd_set, cmd_id, out, out_max, out_len, &code))
            return code;

        /* 2. 检查总超时 */
        uint32_t elapsed = (uint32_t)(m_driver.millis(m_driver.ctx) - start);
        if (elapsed >= timeout_ms)
            return RS_TIMEOUT;

        /* 3. 继续接收；单次最长 100ms，便于及时复查总超时 */
        uint32_t remain = (uint32_t)(timeout_ms - elapsed);
        serviceReceive(remain > 100 ? 100 : remain);
    }
}

uint8_t RoninS::sendAndWait(uint8_t cmd_set, uint8_t cmd_id, const uint8_t *data,
                            uint16_t data_len, uint8_t *out, uint8_t out_max,
                            uint16_t *out_len)
{
    uint8_t seq[2] = {0, 0};

    if (!sendCmd(cmd_set, cmd_id, data, data_len, seq))
        return RS_ERROR;

    return waitResponse(seq, cmd_set, cmd_id, out, out_max, out_len, m_timeoutMs);
}
