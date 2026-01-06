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


    cmd[0] = SOF;                        //SOF

    cmd[1] = cmd_length;                                                                     //根据文档这里是uint16_t，前10bit是帧长度，由于现阶段很难超过256字节偷懒只用前8bit
    cmd[2] = 0x00;

    cmd[3] = cmd_type;

    cmd[4] = enc;                         // enc                                            //编码

    cmd[5] = 0x00;                        // res1                                           //保留位
    cmd[6] = 0x00;                        // res2
    cmd[7] = 0x00;                        // res3

    cmd[8] = seqnum[0];                   // seqnum [1:]                                    //序列号
    cmd[9] = seqnum[1];                   // seqnum [0:1]


    crc16  = crc16_init();
    crc16  = crc16_update(crc16, cmd, 10);
    crc16  = crc16_finalize(crc16);                                                             //crc16校验
    cmd[10] = (crc16 & 0xff);
    cmd[11] = ((crc16 >> 8) & 0xff);
    cmd[12] = cmd_set;
    cmd[13] = cmd_id;                     i = 14;

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


    return cmd;
}


void seq_num(uint8_t *seq_out) {
    static uint16_t Seq_Init_Data = 0;
    if (Seq_Init_Data >= 0xFFFF) {
        Seq_Init_Data = 0x00;
    }
    Seq_Init_Data++;
    seq_out[0] = ((uint8_t*)&Seq_Init_Data)[1];
    seq_out[1] = ((uint8_t*)&Seq_Init_Data)[0];
}

