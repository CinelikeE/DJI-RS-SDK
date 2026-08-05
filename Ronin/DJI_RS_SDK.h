/**
 * @file DJI_RS_SDK.h
 * @brief DJI RS 手持云台 SDK 对外接口（头文件）
 *
 * 封装了 DJI RS SDK 协议文档 2.3.x 章节的主要命令：
 *  - 云台位置/速度控制、角度与限位查询、电机力度、参数推送
 *  - 模块版本、用户参数（TLV）、工作模式、自动校准、智能跟随
 *  - 跟焦器电机控制、相机动作控制
 *
 * 使用前先调用 dji_rs_sdk_init()，再通过 rs_connect() 确认链路。
 *
 * 使用约定：
 *  - 返回 uint8_t 的接口返回“云台返回码”（0x00 成功），失败时返回
 *    本地错误码 RS_TIMEOUT(0x03)/RS_ERROR(0x04)；
 *  - 角度参数/返回值单位一般为 0.1°（0.1°/s 为速度单位）；
 *  - 应答等待基于全局应答队列 rs_res_mq，同一时刻仅支持单命令流。
 */
#ifndef DJI_RS_SDK_H
#define DJI_RS_SDK_H

#include "rtthread.h"
#include "CmdCombine.h"
#include "Handle.h"
#include "DJI_RS_Set.h"
#include "FrameTransmit.h"
#include "CmdParse.h"
#include "Response.h"

#include "stdbool.h"
#include "string.h"

/* 本地错误码（区别于云台返回码；云台返回码见 Response.h 的 enum ReturnCode） */
enum ErrorCode {
    RS_TIMEOUT = 0x03,  /* 等待云台应答超时 */
    RS_ERROR   = 0x04   /* 本地错误：参数非法、发送失败或应答数据不完整 */
};

/* 控制标志字节使用的位定义（position_ctrl_byte / speed_ctrl_byte） */
enum FLAG {
    BIT0 = 0x01,  /* position: 0=增量控制，1=绝对控制；speed: 保留 */
    BIT1 = 0x02,  /* position: Yaw 轴无效标志（1=无效） */
    BIT2 = 0x04,  /* position: Roll 轴无效标志（1=无效） */
    BIT3 = 0x08,  /* position: Pitch 轴无效标志；speed: 1=移动速度不考虑镜头焦距 */
    BIT4 = 0x10,  /* 保留 */
    BIT5 = 0x20,  /* 保留 */
    BIT6 = 0x40,  /* 保留 */
    BIT7 = 0x80   /* speed: 速度控制权（1=接手机台速度控制） */
};

/* 云台轴类型（位置控制与角度查询通用） */
enum AxisType {
    YAW = 0,     /* 偏航轴 */
    ROLL = 1,    /* 横滚轴 */
    PITCH = 2    /* 俯仰轴 */
};

/* 位置控制模式 */
enum MoveMode {
    INCREMENTAL_CONTROL = 0,  /* 增量（相对）控制：指令给出相对当前角度的增量 */
    ABSOLUTE_CONTROL = 1      /* 绝对控制：指令给出目标绝对角度 */
};

/* 速度控制权 */
enum SpeedControl {
    sDISABLED = 0,  /* 释放速度控制权 */
    sENABLED = 1    /* 接手机台速度控制权 */
};

/* 移动速度是否考虑镜头焦距影响 */
enum FocalControl {
    fENABLED = 0,   /* 移动速度考虑相机焦距影响 */
    fDISABLED = 1   /* 移动速度不考虑相机焦距影响 */
};

/* 2.3.4.3 获取云台信息：角度类型 */
enum GimbalAngleType {
    GIMBAL_ATTITUDE_ANGLE = 0x01,  /* 姿态角（云台整体姿态） */
    GIMBAL_JOINT_ANGLE    = 0x02   /* 关节角（各轴电机角度） */
};

/* 2.3.4.14 设置手持云台工作模式 */
enum GimbalWorkMode {
    GIMBAL_MODE_KEEP = 0xFE  /* 保持当前工作模式不变 */
};

/* 2.3.4.14 云台方向（横拍/竖拍） */
enum GimbalOrientation {
    ORIENT_HORIZONTAL_0    = 0x01,  /* 绕 X 轴旋转 0° 的横拍 */
    ORIENT_HORIZONTAL_180  = 0x02,  /* 绕 X 轴旋转 180° 的横拍 */
    ORIENT_VERTICAL_90     = 0x03,  /* 绕 X 轴旋转 90° 的竖拍 */
    ORIENT_VERTICAL_NEG90  = 0x04,  /* 绕 X 轴旋转 -90° 的竖拍 */
    ORIENT_TOGGLE          = 0x05,  /* 横竖拍切换，角度由云台自适应 */
    ORIENT_DEFAULT         = 0xFF   /* 恢复默认模式 */
};

/* 2.3.4.15 设置云台回中、自拍和跟随模式 */
enum GimbalFollowMode {
    FOLLOW_MODE_LOCKED     = 0x00,  /* 云台锁定模式：Yaw/Roll/Pitch 均锁定 */
    FOLLOW_MODE_YAW_FOLLOW = 0x02,  /* Yaw 跟随模式 */
    FOLLOW_MODE_SPORT      = 0x03   /* 运动模式 */
};

/* 2.3.4.15 回中/自拍动作 */
enum CenterSelfieCmd {
    CENTER_CMD  = 0x01,  /* 云台回中 */
    SELFIE_CMD  = 0x02   /* 自拍 */
};

/* 2.3.5.1 第三方相机动作命令 */
enum CameraAction {
    CAMERA_PHOTO             = 0x0001,  /* 拍照 */
    CAMERA_STOP_PHOTO        = 0x0002,  /* 停止拍照 */
    CAMERA_RECORD_START      = 0x0003,  /* 开始录像 */
    CAMERA_RECORD_STOP       = 0x0004,  /* 停止录像 */
    CAMERA_FOCUS_CENTER      = 0x0005,  /* 中心对焦 */
    CAMERA_FOCUS_CENTER_STOP = 0x000B   /* 停止中心对焦 */
};

/**
 * @brief 初始化 SDK 协议配置（使用前调用一次）
 *
 * 设置默认控制标志、加密类型与帧头 CmdType：
 *  - 位置控制：绝对控制模式，Yaw/Roll/Pitch 三轴均有效
 *  - 速度控制：默认禁用
 *  - 加密：不加密（RS_ENC_NONE）
 *  - 帧类型：命令帧，要求云台应答（RS_RESP_MUST）
 */
void dji_rs_sdk_init(void);

/**
 * @brief 连接云台
 * @return true=链路正常，false=查询失败或超时
 * @note 通过查询模块 0x00000001 的版本号确认链路是否可用
 */
bool rs_connect(void);

/**
 * @brief 断开云台
 * @return true=成功，false=失败
 * @note 发送“禁用参数推送”命令，停止云台数据推送
 */
bool rs_disconnect(void);

/**
 * @brief 2.3.4.1 发送云台位置控制命令（仅发送，不等待应答）
 * @param seq       回填本帧序列号（2 字节），可用于之后匹配应答
 * @param yaw_angle   目标偏航角（度），范围 -180.0 ~ 180.0
 * @param roll_angle  目标横滚角（度），范围 -180.0 ~ 180.0
 * @param pitch_angle 目标俯仰角（度），范围 -180.0 ~ 180.0
 * @param time_s      运动时间（秒），范围 0.1 ~ 25.5
 * @return true=发送成功，false=参数越界或发送失败
 * @note 与 move_to() 的区别：本函数只组包并发送，不等待云台应答，
 *       适合 50ms 等周期控制；需要确认云台执行结果请使用 move_to()。
 */
bool rs_send_move_to(uint8_t *seq, float yaw_angle, float roll_angle, float pitch_angle, float time_s);

/**
 * @brief 2.3.4.1 手持云台位置控制（等待应答）
 * @param yaw_angle   目标偏航角（度），范围 -180.0 ~ 180.0
 * @param roll_angle  目标横滚角（度），范围 -180.0 ~ 180.0
 * @param pitch_angle 目标俯仰角（度），范围 -180.0 ~ 180.0
 * @param time_s      运动时间（秒），范围 0.1 ~ 25.5
 * @return 云台返回码（0x00 成功），或 RS_TIMEOUT/RS_ERROR
 * @note 内部先调用 rs_send_move_to() 发送命令，再等待云台应答；
 *       只发送不等待请使用 rs_send_move_to()。
 */
uint8_t move_to(float yaw_angle, float roll_angle, float pitch_angle, float time_s);

/**
 * @brief 设置位置控制中各轴是否有效
 * @param axis  轴类型（YAW / ROLL / PITCH）
 * @param valid true=该轴参与位置控制，false=该轴无效
 * @return true=设置成功，false=轴类型无效
 */
bool set_axis_valid(enum AxisType axis, bool valid);

/**
 * @brief 设置位置控制模式
 * @param type INCREMENTAL_CONTROL=增量（相对）控制，ABSOLUTE_CONTROL=绝对控制
 * @return true
 */
bool set_move_mode(enum MoveMode type);

/**
 * @brief 2.3.4.2 手持云台速度控制（等待应答）
 * @param yaw_speed   偏航速度（0.1°/s），范围 0 ~ 3600
 * @param roll_speed  横滚速度（0.1°/s），范围 0 ~ 3600
 * @param pitch_speed 俯仰速度（0.1°/s），范围 0 ~ 3600
 * @return true=云台返回成功，false=参数越界或发送/应答失败
 */
bool set_speed(int16_t yaw_speed, int16_t roll_speed, int16_t pitch_speed);

/**
 * @brief 设置速度控制权与移动速度是否考虑镜头焦距
 * @param speed_type sENABLED=接手机台速度控制权，sDISABLED=释放
 * @param focal_type fENABLED=考虑镜头焦距影响，fDISABLED=不考虑
 * @return true
 */
bool set_speed_mode(enum SpeedControl speed_type, enum FocalControl focal_type);

/**
 * @brief 2.3.4.3 获取手持云台角度信息（等待应答）
 * @param data_type GIMBAL_ATTITUDE_ANGLE=姿态角，GIMBAL_JOINT_ANGLE=关节角
 * @param yaw   输出偏航角（0.1°），可传 NULL 跳过
 * @param roll  输出横滚角（0.1°），可传 NULL 跳过
 * @param pitch 输出俯仰角（0.1°），可传 NULL 跳过
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t get_gimbal_angle(uint8_t data_type, int16_t *yaw, int16_t *roll, int16_t *pitch);

/**
 * @brief 2.3.4.4 设置手持云台限位角度（等待应答）
 * @param pitch_max 俯仰上限（度），0 ~ 179
 * @param pitch_min 俯仰下限（度），0 ~ 179
 * @param yaw_max   偏航上限（度），0 ~ 179
 * @param yaw_min   偏航下限（度），0 ~ 179
 * @param roll_max  横滚上限（度），0 ~ 179
 * @param roll_min  横滚下限（度），0 ~ 179
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t set_limit_angle(uint8_t pitch_max, uint8_t pitch_min, uint8_t yaw_max, uint8_t yaw_min, uint8_t roll_max, uint8_t roll_min);

/**
 * @brief 2.3.4.5 获取手持云台限位角度（等待应答）
 * @param pitch_max/pitch_min/yaw_max/yaw_min/roll_max/roll_min
 *        输出各轴限位角度（度），均可传 NULL 跳过
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t get_limit_angle(uint8_t *pitch_max, uint8_t *pitch_min, uint8_t *yaw_max, uint8_t *yaw_min, uint8_t *roll_max, uint8_t *roll_min);

/**
 * @brief 2.3.4.6 设置手持云台电机力度（等待应答）
 * @param pitch 俯仰电机力度（0 ~ 100）
 * @param roll  横滚电机力度（0 ~ 100）
 * @param yaw   偏航电机力度（0 ~ 100）
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t set_motor_stiffness(uint8_t pitch, uint8_t roll, uint8_t yaw);

/**
 * @brief 2.3.4.7 获取手持云台电机力度（等待应答）
 * @param pitch 输出俯仰电机力度（0 ~ 100）
 * @param yaw   输出偏航电机力度（0 ~ 100）
 * @param roll  输出横滚电机力度（0 ~ 100）
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t get_motor_stiffness(uint8_t *pitch, uint8_t *yaw, uint8_t *roll);

/**
 * @brief 2.3.4.8 手持云台参数推送设置（等待应答）
 * @param ctrl 0x00=无操作，0x01=使能参数推送，0x02=禁用参数推送
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t set_param_push(uint8_t ctrl);

/**
 * @brief 2.3.4.10 获取模块版本号（等待应答）
 * @param device_id 设备 ID（云台模块通常为 0x00000001）
 * @param version   输出版本号（0xAABBCCDD 表示 AA.BB.CC.DD），可传 NULL 跳过
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t get_module_version(uint32_t device_id, uint32_t *version);

/**
 * @brief 2.3.4.11 摇杆控制命令推送（无应答）
 * @param pitch_speed 俯仰速度（0.1°/s）
 * @param roll_speed  横滚速度（0.1°/s）
 * @param yaw_speed   偏航速度（0.1°/s）
 */
void joystick_control(int16_t pitch_speed, int16_t roll_speed, int16_t yaw_speed);

/**
 * @brief 2.3.4.11 拨轮控制命令推送（无应答）
 * @param dial_speed 拨轮速度（0.1°/s）
 */
void dial_control(int16_t dial_speed);

/**
 * @brief 2.3.4.12 获取手持云台用户参数（TLV，等待应答）
 * @param read_ids 需要读取的用户参数 ID 数组
 * @param id_num   ID 数量
 * @param tlv_buf  输出 TLV 数据缓冲
 * @param tlv_len  输出实际 TLV 数据长度
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t get_user_param(uint8_t *read_ids, uint8_t id_num, uint8_t *tlv_buf, uint8_t *tlv_len);

/**
 * @brief 2.3.4.13 设置手持云台用户参数（TLV，等待应答）
 * @param tlv_id     用户参数 ID
 * @param tlv_length 参数数据长度（最大 32）
 * @param tlv_data   参数数据
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t set_user_param(uint8_t tlv_id, uint8_t tlv_length, uint8_t *tlv_data);

/**
 * @brief 2.3.4.14 设置手持云台工作模式（等待应答）
 * @param work_mode   工作模式（GIMBAL_MODE_KEEP=保持当前模式）
 * @param orientation 云台方向（见 enum GimbalOrientation）
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t set_gimbal_work_mode(uint8_t work_mode, uint8_t orientation);

/**
 * @brief 2.3.4.15 云台回中/自拍（等待应答）
 * @param action CENTER_CMD=回中，SELFIE_CMD=自拍
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t set_gimbal_center_selfie(uint8_t action);

/**
 * @brief 2.3.4.15 设置云台跟随模式（等待应答）
 * @param mode 跟随模式（见 enum GimbalFollowMode）
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t set_gimbal_follow_mode(uint8_t mode);

/**
 * @brief 2.3.4.16 设置自动校准（无应答）
 * @param enable 0=关闭校准，1=开启校准
 * @param type   校准类型
 * @return 云台返回码（仅判断是否成功发送），或 RS_ERROR
 */
uint8_t set_auto_calibration(uint8_t enable, uint8_t type);

/**
 * @brief 2.3.4.18 智能跟随（无应答）
 * @param cmd 智能跟随命令（当前仅支持 0x03）
 * @return 云台返回码（仅判断是否成功发送），或 RS_ERROR
 */
uint8_t set_smart_follow(uint8_t cmd);

/**
 * @brief 2.3.4.19 跟焦器电机位置控制（无应答）
 * @param position 目标位置
 */
void focus_motor_position_control(uint16_t position);

/**
 * @brief 2.3.4.19 跟焦器电机校准（等待应答）
 * @param motor_type 电机类型
 * @param cmd        校准命令
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t focus_motor_calibrate(uint8_t motor_type, uint8_t cmd);

/**
 * @brief 2.3.4.19 获取跟焦器电机位置（等待应答）
 * @param motor_type  电机类型
 * @param calib_state 输出校准状态，可传 NULL 跳过
 * @param position    输出位置，可传 NULL 跳过
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t focus_motor_get_position(uint8_t motor_type, uint8_t *calib_state, uint32_t *position);

/**
 * @brief 2.3.5 相机动作控制（等待应答）
 * @param action 相机动作（见 enum CameraAction）
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t camera_action(uint16_t action);

/**
 * @brief 2.3.5 获取相机状态（等待应答）
 * @param status 输出相机状态，可传 NULL 跳过
 * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
 */
uint8_t camera_get_status(uint8_t *status);

#endif /* DJI_RS_SDK_H */
