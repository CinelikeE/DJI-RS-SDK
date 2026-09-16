/**
 * @file CmdParse.cpp
 * @brief DJI RS 协议帧解析实现
 *
 * 状态机流程：
 *  STATE_IDLE      收到一帧 CAN 数据时校验帧头（SOF / ENC / RES）与帧长，
 *                  合法则开始收帧；
 *  STATE_RECEIVING 持续累积数据，缓冲达到 16 字节时校验 CRC16，
 *                  收满整帧时校验 CRC32；
 *  STATE_COMPLETE  CRC 全部通过，提取字段后清空缓冲，等待下一帧。
 */
#include "CmdParse.h"

CmdParse::CmdParse()
    : m_len(0),
      m_lenMax(0),
      m_state(STATE_IDLE)
{
    reset();
}

void CmdParse::reset()
{
    for (uint16_t i = 0; i < sizeof(m_buffer); ++i)
        m_buffer[i] = 0x00;
    m_len = 0;
    m_lenMax = 0;
    m_state = STATE_IDLE;
}

bool CmdParse::feed(const uint8_t *data, uint8_t len, uint8_t enc, RsMessage &out)
{
    uint8_t buf[8];
    uint8_t i;
    crc16_t crc16;
    crc32_t crc32;

    if (data == nullptr || len == 0)
        return false;
    if (len > sizeof(buf))
        len = (uint8_t)sizeof(buf);

    /* 统一按 8 字节处理，短帧剩余部分补 0，避免越界读取 */
    for (i = 0; i < sizeof(buf); ++i)
        buf[i] = (i < len) ? data[i] : 0x00;

    switch (m_state)
    {
    case STATE_IDLE:
        /* 校验帧头：SOF = 0xAA、ENC 与本机一致、RES 保留位全 0 */
        if ((buf[0] == RS_SOF) && (buf[4] == enc) &&
            (buf[5] == 0x00) && (buf[6] == 0x00) && (buf[7] == 0x00))
        {
            uint16_t frame_len = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
            if (frame_len >= RS_FRAME_MIN_LEN && frame_len <= RS_MAX_FRAME_LEN)
            {
                m_lenMax = frame_len;
                for (i = 0; i < 8; ++i)
                    m_buffer[i] = buf[i];
                m_len = 8;
                m_state = STATE_RECEIVING;
            }
        }
        break;

    case STATE_RECEIVING:
        /* 追加本帧数据（带越界保护） */
        for (i = 0; i < len; ++i)
        {
            if ((uint16_t)(m_len + i) < sizeof(m_buffer))
                m_buffer[m_len + i] = buf[i];
        }
        m_len = (uint16_t)(m_len + len);

        /* 帧头 + CRC16 到齐后校验一次 CRC16 */
        if (m_len >= 16 && (uint16_t)(m_len - len) < 16)
        {
            crc16 = crc16_init();
            crc16 = crc16_update(crc16, m_buffer, RS_FRAME_HEAD_LEN);
            crc16 = crc16_finalize(crc16);
            if ((m_buffer[10] != (uint8_t)(crc16 & 0xFF)) ||
                (m_buffer[11] != (uint8_t)((crc16 >> 8) & 0xFF)))
            {
                reset(); /* 校验失败，丢弃整帧 */
                break;
            }
        }

        /* 收满整帧后校验 CRC32 */
        if (m_lenMax > 0 && m_len >= m_lenMax)
        {
            crc32 = crc32_init();
            crc32 = crc32_update(crc32, m_buffer, (size_t)(m_lenMax - 4));
            crc32 = crc32_finalize(crc32);
            if ((m_buffer[m_lenMax - 4] != (uint8_t)(crc32 & 0xFF)) ||
                (m_buffer[m_lenMax - 3] != (uint8_t)((crc32 >> 8) & 0xFF)) ||
                (m_buffer[m_lenMax - 2] != (uint8_t)((crc32 >> 16) & 0xFF)) ||
                (m_buffer[m_lenMax - 1] != (uint8_t)((crc32 >> 24) & 0xFF)))
            {
                reset(); /* 校验失败，丢弃整帧 */
                break;
            }
            m_state = STATE_COMPLETE;
        }
        break;

    case STATE_COMPLETE:
    default:
        break;
    }

    if (m_state != STATE_COMPLETE)
        return false;

    makeMessage(out);
    reset();
    return true;
}

void CmdParse::makeMessage(RsMessage &out) const
{
    uint16_t frame_len = (uint16_t)m_buffer[1] | ((uint16_t)m_buffer[2] << 8);
    uint16_t data_seg_len = (frame_len > 16) ? (uint16_t)(frame_len - 16) : 0; /* CmdSet + CmdID + CmdData */
    uint16_t payload_len = (data_seg_len > 2) ? (uint16_t)(data_seg_len - 2) : 0; /* CmdData */
    uint16_t n;

    out.seq[0] = m_buffer[8];
    out.seq[1] = m_buffer[9];
    out.frame_type = (uint8_t)((m_buffer[3] >> 5) & 0x01);
    out.cmd_set = m_buffer[12];
    out.cmd_id = m_buffer[13];

    n = (payload_len > sizeof(out.data)) ? (uint16_t)sizeof(out.data) : payload_len;
    out.len = n;
    for (uint16_t i = 0; i < n; ++i)
        out.data[i] = m_buffer[14 + i];
    for (uint16_t i = n; i < sizeof(out.data); ++i)
        out.data[i] = 0x00;
}

void CmdParse::parsePushParam(const RsMessage &msg, RsPushParam &out)
{
    const uint8_t *d;

    out = RsPushParam(); /* 先清零，避免残留旧数据 */

    if (msg.len < 22)
        return;
    d = msg.data;

    out.valid_flag = d[0];
    out.yaw_angle = (int16_t)((uint16_t)d[1] | ((uint16_t)d[2] << 8));
    out.roll_angle = (int16_t)((uint16_t)d[3] | ((uint16_t)d[4] << 8));
    out.pitch_angle = (int16_t)((uint16_t)d[5] | ((uint16_t)d[6] << 8));
    out.yaw_joint_angle = (int16_t)((uint16_t)d[7] | ((uint16_t)d[8] << 8));
    out.roll_joint_angle = (int16_t)((uint16_t)d[9] | ((uint16_t)d[10] << 8));
    out.pitch_joint_angle = (int16_t)((uint16_t)d[11] | ((uint16_t)d[12] << 8));
    out.pitch_max = d[13];
    out.pitch_min = d[14];
    out.yaw_max = d[15];
    out.yaw_min = d[16];
    out.roll_max = d[17];
    out.roll_min = d[18];
    out.pitch_stiffness = d[19];
    out.yaw_stiffness = d[20];
    out.roll_stiffness = d[21];
}
