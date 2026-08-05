/**
 * @file Response.h
 * @brief 应答帧等待与“发送并等待应答”接口（头文件）
 *
 * 协议文档 2.3.2 定义了云台返回码：
 *  - 0x00 执行成功
 *  - 0x01 解析失败
 *  - 0x02 执行失败
 *  - 0xFF 未知错误
 */
#ifndef SRC_RONIN_RESPONSE_H_
#define SRC_RONIN_RESPONSE_H_

#include "rtthread.h"
#include "CmdParse.h"
#include "FrameTransmit.h"

/* 文档 2.3.2 返回码 */
enum ReturnCode {
    EXECUTION_SUCCESSFUL = 0,
    PARSE_ERROR = 1,
    EXECUTION_FAILS = 2,
    UNDEFINED_ERROR = 0xFF
};

/* 等待指定 SEQ/命令的应答帧，out[0] 为返回码，返回码错误时返回 0x03/0x04 */
uint8_t rs_wait_response(uint8_t seq[2], uint8_t cmd_set, uint8_t cmd_id,
                         uint8_t *out, uint8_t out_max, uint8_t *out_len,
                         uint32_t timeout_ms);

/* 发送命令帧并等待应答（适用于文档中带应答帧的命令） */
uint8_t rs_send_and_wait(uint8_t cmd_set, uint8_t cmd_id,
                         uint8_t *payload, uint8_t payload_len,
                         uint8_t *out, uint8_t out_max, uint8_t *out_len,
                         uint32_t timeout_ms);

#endif /* SRC_RONIN_RESPONSE_H_ */
