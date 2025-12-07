#include "CmdCombine.h"

uint8_t seqnum[2];
uint8_t i = 0;
crc16_t crc16;
crc32_t crc32;

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
 * @brief 生成SDK单帧数据帧，注意：调用此函数后必须rt_free掉
 * @param cmd_type:应答和帧类型
 * @param cmd_set:命令集
 * @param cmd_id:命令码
 * @param data：数据内容
 * @return 单帧数据帧指针
 */
uint8_t *Combine(uint8_t cmd_type, uint8_t cmd_set, uint8_t cmd_id, uint8_t *data, uint8_t data_length){

    uint8_t cmd_length = 18 + data_length ;// 10byte prefix + 2byte crc16 + (n+2)byte data + 4byte crc32


    seq_num(seqnum);

    uint8_t *cmd = (uint8_t*) rt_malloc(cmd_length);                                     //申请一段18+n byte的内存地址


    cmd[i] = 0xAA;                       i++;//SOF                                              //帧头           固定值0xAA

    cmd[i] = cmd_length;                 i++;                                                   //帧长度，按文档为后10bit，由于LSB First，且帧长一般不大可能超过255，可直接用1字节表达
    cmd[i] = 0x00;                       i++;                                                   //版本号，仅前6bit，一般为0，由于后2bit几乎不可能为1，整个字节直接为0

    cmd[i] = cmd_type;                   i++;                                                   //指令类型

    cmd[i] = 0x00;                       i++; // enc                                            //加密类型，暂且保留为0

    cmd[i] = 0x00;                       i++; // res1                                           //保留位
    cmd[i] = 0x00;                       i++; // res2
    cmd[i] = 0x00;                       i++; // res3

    cmd[i] = seqnum[0];                  i++; // seqnum [1:]                                    //序列号，前后两帧序列号必须不同
    cmd[i] = seqnum[1];                  i++; // seqnum [0:1]


    crc16  = crc16_init();
    crc16  = crc16_update(crc16, cmd,i);                                                        //crc16校验
    //cmd[i] = ((uint8_t*)&crc16)[0];      i++; // crc16 [1:]
    cmd[i] = (crc16 & 0xff);             i++;
    //cmd[i] = ((uint8_t*)&crc16)[1];      i++; // crc16 [0:1]
    cmd[i] = ((crc16 >> 8) & 0xff);      i++;
    cmd[i] = cmd_set;                    i++; // crc16 [0:1]
    cmd[i] = cmd_id;                     i++; // crc16 [0:1]

    for (size_t j = 0; j < data_length; j++)
    {
        cmd[i] = data[j];                i++;                                                    //write CmdData byte  写指令字节
    }

                                                                              //crc32校验
    crc32  = crc32_init();
    crc32  = crc32_update(crc32, cmd, i);
    crc32  = crc32_finalize(crc32);

    //cmd[i] = ((uint8_t*)&crc32)[0];      i++; // crc32 [3:]
    cmd[i] = (crc32 & 0xff);             i++;
    //cmd[i] = ((uint8_t*)&crc32)[1];      i++; // crc32 [2:3]
    cmd[i] = ((crc32 >> 8) & 0xff);      i++;
    //cmd[i] = ((uint8_t*)&crc32)[2];      i++; // crc32 [1:2]
    cmd[i] = ((crc32 >> 16) & 0xff);     i++;
    //cmd[i] = ((uint8_t*)&crc32)[3];           // crc32 [0:1]
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
    // 直接写入输出参数，避免 malloc/free
    seq_out[0] = ((uint8_t*)&Seq_Init_Data)[1];
    seq_out[1] = ((uint8_t*)&Seq_Init_Data)[0];
}

