/**
 * @file CmdCombine.c
 * @brief DJI RS 协议数据帧合成实现
 *
 * 主要函数：
 *  - Combine()：按协议将各字段打包为完整数据帧（内部 rt_malloc 申请内存，调用方需 rt_free）
 *  - seq_num()：生成自增的 2 字节帧序列号
 *
 * 说明：协议帧长度字段占 10 位，因此理论上最大帧长可达 1023 字节；
 * 本实现为配合接收缓冲，将帧长上限限制为 RS_MAX_FRAME_LEN(256)。
 */
#include "CmdCombine.h"
#include "rtthread.h"
#include "DJI_RS_Set.h"

/* DJI RS SDK Protocol Description
 *
 * 2.1 Data Format
 * +----------------------------------------------+------+------+------+
 * |                     PREFIX                   | CRC  | DATA | CRC  |
 * |------+----------+-------+------+------+------+------+------+------|
 * |SOF   |Ver/Length|CmdType|ENC   |RES   |SEQ   |CRC-16|DATA  |CRC-32|
 * |------|----------|-------|------|------|------|------|------|------|
 * |1-byte|2-byte    |1-byte |1-byte|3-byte|2-byte|2-byte|n-byte|4-byte|
 * +------+----------+-------+------+------+------+------+------+------+
 *
 * 2.2 Data Segment (field DATA in 2.1 Data Format)
 * +---------------------+
 * |           DATA      |
 * |------+------+-------|
 * |CmdSet|CmdID |CmdData|
 * |------|------|-------|
 * |1-byte|1-byte|n-byte |
 * +------+------+-------+
 */

/**
 * @brief DJI_RS_SDK 数据帧指令合成，由于函数内 malloc 了一段地址调用后需要立即 rt_free 掉
 * @return 合成的数据帧数组
 */
uint8_t *Combine(uint8_t cmd_type, uint8_t cmd_set, uint8_t cmd_id, uint8_t *data, uint16_t data_length)
{
    uint16_t cmd_length = 18 + data_length;  /* 10 前缀 + 2 crc16 + (2+n) 数据段 + 4 crc32 */
    uint16_t ver_length = cmd_length & 0x03FF; /* [15:10] 版本号默认 0，[9:0] 帧长度 */
    uint8_t seqnum[2];
    uint8_t i = 0;
    crc16_t crc16;
    crc32_t crc32;
    uint8_t *cmd;

    if (cmd_length > RS_MAX_FRAME_LEN)
        return RT_NULL;

    seq_num(seqnum);

    cmd = (uint8_t *)rt_malloc(cmd_length);
    if (cmd == RT_NULL)
        return RT_NULL;

    cmd[0] = SOF;                        /* 帧头，固定 0xAA */
    cmd[1] = ver_length & 0xFF;          /* Ver/Length，LSB first */
    cmd[2] = (ver_length >> 8) & 0xFF;
    cmd[3] = cmd_type;                   /* 指令类型 */
    cmd[4] = enc;                        /* 加密 */
    cmd[5] = 0x00;                       /* res1 */
    cmd[6] = 0x00;                       /* res2 */
    cmd[7] = 0x00;                       /* res3 */
    cmd[8] = seqnum[0];                  /* 序列号 */
    cmd[9] = seqnum[1];

    crc16 = crc16_init();
    crc16 = crc16_update(crc16, cmd, 10);
    crc16 = crc16_finalize(crc16);
    cmd[10] = crc16 & 0xFF;
    cmd[11] = (crc16 >> 8) & 0xFF;

    cmd[12] = cmd_set;
    cmd[13] = cmd_id;
    i = 14;
    for (size_t j = 0; j < data_length; j++)
    {
        cmd[i] = data[j];
        i++;
    }

    crc32 = crc32_init();
    crc32 = crc32_update(crc32, cmd, i);
    crc32 = crc32_finalize(crc32);

    cmd[i] = crc32 & 0xFF;
    i++;
    cmd[i] = (crc32 >> 8) & 0xFF;
    i++;
    cmd[i] = (crc32 >> 16) & 0xFF;
    i++;
    cmd[i] = (crc32 >> 24) & 0xFF;

    return cmd;
}

void seq_num(uint8_t *seq_out)
{
    static uint16_t Seq_Init_Data = 0; /* 静态序列号计数器，函数每次调用自增 1 */
    if (Seq_Init_Data >= 0xFFFF)
        Seq_Init_Data = 0x00;          /* 溢出后回绕，从 0 重新开始 */
    Seq_Init_Data++;
    seq_out[0] = (uint8_t)((Seq_Init_Data >> 8) & 0xFF); /* 序列号高字节 */
    seq_out[1] = (uint8_t)(Seq_Init_Data & 0xFF);        /* 序列号低字节 */
}
