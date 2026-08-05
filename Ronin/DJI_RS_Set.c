/**
 * @file DJI_RS_Set.c
 * @brief DJI RS SDK 协议配置项实现
 *
 * 维护位置/速度控制标志字节、加密类型和 CmdType 字节，
 * 供帧合成（CmdCombine）与上层业务函数使用。
 */
#include "DJI_RS_Set.h"

uint8_t position_ctrl_byte = 0; /* 位置控制标志：BIT0=绝对/相对，BIT1~BIT3=Yaw/Roll/Pitch 轴有效 */
uint8_t speed_ctrl_byte    = 0; /* 速度控制标志：BIT3=是否考虑镜头焦段，BIT7=速度控制杆使能 */
uint8_t enc                = 0; /* 帧头 ENC 字节：加密类型（0=不加密） */
uint8_t CmdType            = 0; /* 帧头 CmdType 字节：响应类型 + 帧类型 */

/**
 * @brief 设置加密类型
 * @param encode 加密类型（RS_ENC_NONE / RS_ENC_AES256）
 */
void Enc_Set(RS_EncType encode)
{
    enc = (uint8_t)encode; /* 直接写入帧头 ENC 字节 */
}

/* 文档 2.2：CmdType [4:0] 应答类型，[5] 帧类型（0 命令帧 / 1 应答帧） */
/**
 * @brief 设置帧头 CmdType 字节
 * @param response 应答类型（无应答 / 可选应答 / 必须应答）
 * @param type     帧类型（命令帧 / 应答帧）
 */
void CmdType_Set(RS_ResponseType response, RS_FrameType type)
{
    /* 低 5 位存应答类型，bit5 存帧类型，两者合成一个字节 */
    CmdType = ((uint8_t)response & 0x1F) | (((uint8_t)type << 5) & 0x20);
}
