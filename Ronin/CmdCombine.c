#include "CmdCombine.h"



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
 * @brief DJI_RS_SDK数据帧指令合成，由于函数内malloc了一段地址调用后需要立即rt_free掉
 * @return 合成的数据帧数组
 */
uint8_t *Combine(uint8_t cmd_type, uint8_t cmd_set, uint8_t cmd_id, uint8_t *data, uint8_t data_length){

    uint8_t cmd_length = 18 + data_length ;// 10byte prefix + 2byte crc16 + (n+2)byte data + 4byte crc32

    uint8_t seqnum[2];
    uint8_t i = 0;
    crc16_t crc16;
    crc32_t crc32;

    seq_num(seqnum);

    uint8_t *cmd = (uint8_t*) rt_malloc(cmd_length);                                            //帧长度


    cmd[i] = 0xAA;                       i++;//SOF

    cmd[i] = cmd_length;                 i++;                                                   //根据文档这里是uint16_t，前10bit是帧长度，由于现阶段很难超过256字节偷懒只用前8bit
    cmd[i] = 0x00;                       i++;

    cmd[i] = cmd_type;                   i++;

    cmd[i] = ENC;                       i++; // enc                                            //编码

    cmd[i] = 0x00;                       i++; // res1                                           //保留位
    cmd[i] = 0x00;                       i++; // res2
    cmd[i] = 0x00;                       i++; // res3

    cmd[i] = seqnum[0];                  i++; // seqnum [1:]                                    //序列号
    cmd[i] = seqnum[1];                  i++; // seqnum [0:1]


    crc16  = crc16_init();
    crc16  = crc16_update(crc16, cmd,i);
    crc16  = crc16_finalize(crc16);                                                             //crc16校验
    cmd[i] = (crc16 & 0xff);             i++;
    cmd[i] = ((crc16 >> 8) & 0xff);      i++;
    cmd[i] = cmd_set;                    i++;
    cmd[i] = cmd_id;                     i++;

    for (size_t j = 0; j < data_length; j++)
    {
        cmd[i] = data[j];                i++;                                                    //write CmdData byte
    }


    crc32  = crc32_init();
    crc32  = crc32_update(crc32, cmd, i);
    crc32  = crc32_finalize(crc32);                                                             //crc32校验


    cmd[i] = (crc32 & 0xff);             i++;
    cmd[i] = ((crc32 >> 8) & 0xff);      i++;
    cmd[i] = ((crc32 >> 16) & 0xff);     i++;
    cmd[i] = ((crc32 >> 24) & 0xff);

        i  = 0;
    return cmd;
}


void seq_num(uint8_t *seq_out) {
    static uint16_t Seq_Init_Data = 0x2210;
    if (Seq_Init_Data >= 0xFFF0) {
        Seq_Init_Data = 0x0002;
    }
    Seq_Init_Data++;
    seq_out[0] = ((uint8_t*)&Seq_Init_Data)[1];
    seq_out[1] = ((uint8_t*)&Seq_Init_Data)[0];
}

