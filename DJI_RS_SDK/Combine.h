/**
 * @file Combine.h
 * @brief DJI RS 协议帧合成（对应原 C 版 CmdCombine.c / FrameTransmit.c 的组帧部分）
 *
 * 帧结构（协议文档 2.1）：
 * +------+-----------+-------+------+------+------+-------+--------+-------+-------+
 * | SOF  |Ver/Length |CmdType| ENC  | RES  | SEQ  | CRC16 | CmdSet | CmdID | CRC32 |
 * |1-byte|  2-byte   |1-byte |1-byte|3-byte|2-byte| 2-byte| 1-byte |1-byte |4-byte |
 * +------+-----------+-------+------+------+------+-------+--------+-------+-------+
 *                        DATA = CmdSet + CmdID + CmdData
 *
 * 与原 C 实现最大的区别：组帧结果写入调用方提供的缓冲区，
 * 内部不做 rt_malloc / new，避免动态内存带来的可移植性问题。
 */
#ifndef RS_COMBINE_H
#define RS_COMBINE_H

#include <stdint.h>
#include "custom_crc16.h"
#include "custom_crc32.h"

#define RS_SOF            0xAA /* 帧头固定值 */
#define RS_FRAME_HEAD_LEN 10   /* SOF + Ver/Length + CmdType + ENC + RES + SEQ */
#define RS_FRAME_MIN_LEN  18   /* 10 + CRC16(2) + CmdSet(1) + CmdID(1) + CRC32(4) */
#define RS_MAX_FRAME_LEN  256  /* 帧长上限（协议本身支持 10bit 帧长，这里与接收缓冲一致） */

class Combine
{
public:
    Combine();

    /**
     * @brief 合成一条完整的 DJI RS 协议帧
     * @param out          输出缓冲区（由调用方提供）
     * @param out_capacity 输出缓冲区大小
     * @param cmd_type     帧头 CmdType（应答类型 + 帧类型，见 RsResponseType / RsFrameType）
     * @param enc          帧头 ENC（加密类型，0 = 不加密）
     * @param cmd_set      命令集（0x0E = 云台，0x0D = 相机）
     * @param cmd_id       命令 ID
     * @param data         命令数据（无数据时可为 nullptr）
     * @param data_len     命令数据长度
     * @return 帧总长度；0 表示参数非法或输出缓冲区不足
     */
    uint16_t build(uint8_t *out, uint16_t out_capacity,
                   uint8_t cmd_type, uint8_t enc,
                   uint8_t cmd_set, uint8_t cmd_id,
                   const uint8_t *data, uint16_t data_len);

    /** @brief 取下一个 2 字节帧序列号（自增，溢出回绕） */
    void nextSeq(uint8_t *seq_out);

private:
    uint16_t m_seq; /* 帧序列号计数器 */
};

#endif /* RS_COMBINE_H */
