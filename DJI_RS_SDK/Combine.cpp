/**
 * @file Combine.cpp
 * @brief DJI RS 协议帧合成实现
 *
 * 组帧步骤（协议 2.1）：
 *  1. 填写 10 字节帧头：SOF、Ver/Length、CmdType、ENC、RES(3)、SEQ(2)；
 *  2. 对帧头 10 字节计算 CRC16，写入第 10、11 字节（小端）；
 *  3. 填写数据段：CmdSet + CmdID + CmdData；
 *  4. 对除尾部 CRC32 外的所有数据计算 CRC32，写入最后 4 字节（小端）。
 */
#include "Combine.h"

Combine::Combine()
    : m_seq(0)
{
}

void Combine::nextSeq(uint8_t *seq_out)
{
    if (seq_out == nullptr)
        return;

    if (m_seq >= 0xFFFF)
        m_seq = 0x0000; /* 溢出回绕，从 0 重新开始 */
    m_seq++;

    seq_out[0] = (uint8_t)((m_seq >> 8) & 0xFF); /* 序列号高字节 */
    seq_out[1] = (uint8_t)(m_seq & 0xFF);        /* 序列号低字节 */
}

uint16_t Combine::build(uint8_t *out, uint16_t out_capacity,
                        uint8_t cmd_type, uint8_t enc,
                        uint8_t cmd_set, uint8_t cmd_id,
                        const uint8_t *data, uint16_t data_len)
{
    uint16_t frame_len = (uint16_t)(RS_FRAME_MIN_LEN + data_len);
    uint16_t ver_length = (uint16_t)(frame_len & 0x03FF); /* [15:10] 版本号固定 0，[9:0] 帧长 */
    uint8_t  seq[2];
    uint16_t i;
    crc16_t  crc16;
    crc32_t  crc32;

    if (out == nullptr)
        return 0;
    if (data_len > 0 && data == nullptr)
        return 0;
    if (frame_len > out_capacity || frame_len > RS_MAX_FRAME_LEN)
        return 0;

    nextSeq(seq);

    out[0] = RS_SOF;                       /* 帧头固定 0xAA */
    out[1] = (uint8_t)(ver_length & 0xFF); /* Ver/Length 小端 */
    out[2] = (uint8_t)((ver_length >> 8) & 0xFF);
    out[3] = cmd_type; /* 指令类型 */
    out[4] = enc;      /* 加密类型 */
    out[5] = 0x00;     /* res1 */
    out[6] = 0x00;     /* res2 */
    out[7] = 0x00;     /* res3 */
    out[8] = seq[0];   /* 序列号 */
    out[9] = seq[1];

    /* 帧头 CRC16 */
    crc16 = crc16_init();
    crc16 = crc16_update(crc16, out, RS_FRAME_HEAD_LEN);
    crc16 = crc16_finalize(crc16);
    out[10] = (uint8_t)(crc16 & 0xFF);
    out[11] = (uint8_t)((crc16 >> 8) & 0xFF);

    /* 数据段：CmdSet + CmdID + CmdData */
    out[12] = cmd_set;
    out[13] = cmd_id;
    for (i = 0; i < data_len; ++i)
        out[14 + i] = data[i];

    /* 整帧 CRC32（除尾部 4 字节） */
    i = (uint16_t)(14 + data_len);
    crc32 = crc32_init();
    crc32 = crc32_update(crc32, out, i);
    crc32 = crc32_finalize(crc32);
    out[i++] = (uint8_t)(crc32 & 0xFF);
    out[i++] = (uint8_t)((crc32 >> 8) & 0xFF);
    out[i++] = (uint8_t)((crc32 >> 16) & 0xFF);
    out[i++] = (uint8_t)((crc32 >> 24) & 0xFF);

    return frame_len;
}
