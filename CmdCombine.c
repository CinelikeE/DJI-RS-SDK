#include "CmdCombine.h"
#include "string.h"

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

uint8_t *Combine(uint8_t cmd_type, uint8_t cmd_set, uint8_t cmd_id, uint8_t *data, uint8_t data_lenth){

    uint16_t cmd_length = 18 + data_lenth ;// 10byte prefix + 2byte crc16 + (n+2)byte data + 4byte crc32

    uint8_t *SeqNum = seq_num();                                                                //
    uint8_t seqnum[2] = {SeqNum[0],SeqNum[1]};                                          //
    free(SeqNum);


    int i = 0;
    uint8_t *cmd = (uint8_t*) calloc(cmd_length, sizeof(uint8_t));   //申请一段18+n byte的内存地址


    cmd[i] = 0xAA; i++;//SOF                                                                    //帧头           固定值0xAA
    cmd[i] = ((uint8_t*)&cmd_length)[0]; i++;                                                   //帧长度和版本号   [9:0]bit为帧长度；[15:10]bit为版本号，一般为0
    cmd[i] = ((uint8_t*)&cmd_length)[1]; i++;
    cmd[i] = cmd_type; i++;                                                                     //指令类型
    cmd[i] = 0x00; i++; // enc                                                                  //加密类型
    cmd[i] = 0x00; i++; // res1                                                                 //保留位
    cmd[i] = 0x00; i++; // res2
    cmd[i] = 0x00; i++; // res3
    cmd[i] = seqnum[0]; i++; // seqnum [1:]                                                     //序列号
    cmd[i] = seqnum[1]; i++; // seqnum [0:1]

    crc16_t crc16;
    crc16 = crc16_init();
    crc16 = crc16_update(crc16, cmd,i);                                       //生成crc16
    cmd[i] = ((uint8_t*)&crc16)[0]; i++; // crc16 [1:]
    cmd[i] = ((uint8_t*)&crc16)[1]; i++; // crc16 [0:1]

    cmd[i] = cmd_set; i++; // crc16 [0:1]
    cmd[i] = cmd_id; i++; // crc16 [0:1]

    for (size_t j = 0; j < data_lenth; j++){                                                   //write CmdData byte  写指令字节
        cmd[i] = data[j]; i++;
    }

    crc32_t crc32;
    crc32 = crc32_init();
    crc32 = crc32_update(crc32, cmd, i);
    crc32 = crc32_finalize(crc32);
    cmd[i] = ((uint8_t*)&crc32)[0]; i++; // crc32 [3:]
    cmd[i] = ((uint8_t*)&crc32)[1]; i++; // crc32 [2:3]
    cmd[i] = ((uint8_t*)&crc32)[2]; i++; // crc32 [1:2]
    cmd[i] = ((uint8_t*)&crc32)[3];      // crc32 [0:1]

    return cmd;
}

uint8_t  *seq_num() {
    static uint16_t Seq_Init_Data = 0x2210;
    if (Seq_Init_Data >= 0xFFF0)
    {
        Seq_Init_Data = 0x0002;
    }
    Seq_Init_Data += 1;

    uint8_t *ret = (uint8_t*) malloc(2);
    ret[0] = ((uint8_t*)&Seq_Init_Data)[1];
    ret[1] = ((uint8_t*)&Seq_Init_Data)[0];

    return ret ;
}

