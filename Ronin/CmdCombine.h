/**
 * @file CmdCombine.h
 * @brief DJI RS 协议数据帧合成模块（头文件）
 *
 * 负责把命令类型、命令集、命令 ID 和命令数据打包成一条符合
 * DJI RS SDK 协议的完整数据帧，并提供帧序列号生成功能。
 *
 * 帧结构：
 * SOF(1) | Ver/Length(2) | CmdType(1) | ENC(1) | RES(3) | SEQ(2)
 *       | CRC16(2) | DATA(2+n) | CRC32(4)
 */
#ifndef CMDCOMBINE_H
#define CMDCOMBINE_H

#include <stdint.h>
#include "custom_crc16.h"
#include "custom_crc32.h"

#define SOF 0xAA

/* 协议帧长上限（与接收缓冲一致；协议本身支持 10 位帧长） */
#define RS_MAX_FRAME_LEN 256

/* 合成一条完整 DJI RS 数据帧（内部 rt_malloc，调用后需立即 rt_free） */
uint8_t *Combine(uint8_t cmd_type, uint8_t cmd_set, uint8_t cmd_id, uint8_t *data, uint16_t data_length);

/* 生成 2 字节序列号（自增，溢出回绕） */
void seq_num(uint8_t *seq_out);

#endif /* CMDCOMBINE_H */
