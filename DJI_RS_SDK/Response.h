/**
 * @file Response.h
 * @brief 云台返回码与本地错误码定义（对应原 Response.h / DJI_RS_SDK.h 中的枚举）
 *
 * 需要应答的命令统一返回 uint8_t：
 *  - 0x00 表示云台执行成功；
 *  - 0x01 / 0x02 / 0xFF 为云台返回的错误码；
 *  - 0x03 / 0x04 为本地错误码（超时、发送失败等）。
 */
#ifndef RS_RESPONSE_H
#define RS_RESPONSE_H

/** @brief 协议 2.3.2 云台返回码 */
enum RsReturnCode
{
    EXECUTION_SUCCESSFUL = 0x00, /* 执行成功 */
    PARSE_ERROR          = 0x01, /* 解析失败 */
    EXECUTION_FAILS      = 0x02, /* 执行失败 */
    UNDEFINED_ERROR      = 0xFF  /* 未知错误 */
};

/** @brief 本地错误码（与云台返回码区分） */
enum RsErrorCode
{
    RS_TIMEOUT = 0x03, /* 等待云台应答超时 */
    RS_ERROR   = 0x04  /* 本地错误：参数非法、发送失败、应答数据不完整 */
};

#endif /* RS_RESPONSE_H */
