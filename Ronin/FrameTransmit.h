/**
 * @file FrameTransmit.h
 * @brief DJI RS 命令帧发送模块（头文件）
 *
 * 提供通用命令帧发送接口 rs_send_cmd()。
 * 发送动作本身由 Handle.c 中的 CAN 发送函数完成。
 * 云台位置控制的发送封装位于 DJI_RS_SDK.c（rs_send_move_to）。
 */
#ifndef _FRAMETRANSMIT_H_
#define _FRAMETRANSMIT_H_

#include "stdbool.h"
#include "CmdCombine.h"
#include "Handle.h"
#include "DJI_RS_Set.h"

/* 通用命令帧发送（不等待应答）：成功返回 true，seq_out 返回帧序列号 */
bool rs_send_cmd(uint8_t cmd_set, uint8_t cmd_id, uint8_t *data, uint16_t data_len, uint8_t *seq_out);

#endif /* _FRAMETRANSMIT_H_ */
