/**
 * @file DJI_RS_SDK.c
 * @brief DJI RS 手持云台 SDK 对外接口实现
 *
 * 每个业务函数对应协议文档 2.3.x 的一条命令：
 *  - 需要应答的命令：通过 rs_send_and_wait() 发送并等待应答，
 *    返回值为云台返回码（0x00 成功）或 RS_TIMEOUT/RS_ERROR；
 *  - 无需应答的命令（摇杆/拨轮、自动校准、智能跟随等）：
 *    通过 rs_send_cmd() 直接发送。
 */
#include "DJI_RS_SDK.h"

/* 等待云台应答的默认超时时间（毫秒） */
#define RS_RESPONSE_TIMEOUT 2000

/* @brief 初始化 SDK 协议配置
 *
 * 设置位置/速度控制标志、加密类型与 CmdType（必须应答 + 命令帧），
 * 在调用任何业务接口前执行一次。 */
void dji_rs_sdk_init(void)
{
    position_ctrl_byte = 0x00;              /* 初始化为绝对模式 */
    position_ctrl_byte |= BIT0;             /* 默认绝对位置控制 */
    speed_ctrl_byte = 0x00;                 /* 初始禁用速度控制 */
    speed_ctrl_byte |= BIT3;                /* 默认不考虑相机焦距影响 */

    Enc_Set(RS_ENC_NONE);
    CmdType_Set(RS_RESP_MUST, RS_FRAME_COMMAND);
}

/* ================= 2.3.4.1 手持云台位置控制 ================= */
/**
 * @brief 2.3.4.1 发送云台位置控制命令（仅发送，不等待应答）
 *
 * 与 move_to() 的区别：本函数只负责组包并发送到 CAN 总线，
 * 不等待云台应答，适合 50ms 等周期控制场景。
 * 需要确认云台执行结果时请改用 move_to()。
 *
 * @param seq       回填本帧序列号（2 字节），可用于之后匹配应答
 * @param yaw_angle   目标偏航角（度），范围 -180.0 ~ 180.0
 * @param roll_angle  目标横滚角（度），范围 -180.0 ~ 180.0
 * @param pitch_angle 目标俯仰角（度），范围 -180.0 ~ 180.0
 * @param time_s      运动时间（秒），协议单位为 0.1 秒，范围 0.1 ~ 25.5
 * @return true=发送成功，false=参数越界或发送失败
 */
bool rs_send_move_to(uint8_t *seq, float yaw_angle, float roll_angle, float pitch_angle, float time_s)
{
    int16_t yaw = (int16_t)(yaw_angle * 10);   /* 角度换算为 0.1° 单位 */
    int16_t roll = (int16_t)(roll_angle * 10);
    int16_t pitch = (int16_t)(pitch_angle * 10);
    uint16_t time_x10 = (uint16_t)(time_s * 10.0f); /* 时间换算为 0.1s 单位 */
    uint8_t data_payload[8];

    /* 校验各轴角度与运动时间是否在协议允许范围内 */
    if (!(yaw >= -1800 && yaw <= 1800 &&
          roll >= -1800 && roll <= 1800 &&
          pitch >= -1800 && pitch <= 1800 &&
          time_x10 >= 1 && time_x10 <= 255))
    {
        rt_kprintf("Error!! DJI_RS_SDK.c Function: rs_send_move_to overflow!\n");
        return false;
    }

    data_payload[0] = yaw & 0xFF;            /* Yaw 低字节 */
    data_payload[1] = (yaw >> 8) & 0xFF;     /* Yaw 高字节 */
    data_payload[2] = roll & 0xFF;           /* Roll 低字节 */
    data_payload[3] = (roll >> 8) & 0xFF;    /* Roll 高字节 */
    data_payload[4] = pitch & 0xFF;          /* Pitch 低字节 */
    data_payload[5] = (pitch >> 8) & 0xFF;   /* Pitch 高字节 */
    data_payload[6] = position_ctrl_byte;    /* 位置控制标志 */
    data_payload[7] = (uint8_t)time_x10;     /* 运动时间（0.1s 单位） */

    return rs_send_cmd(0x0E, 0x00, data_payload, sizeof(data_payload), seq); /* 0x0E 云台命令集，0x00 位置控制 */
}

/* @brief 云台位置控制（等待应答）
 * @return 云台返回码；组包失败返回 RS_ERROR(0x04) */
uint8_t move_to(float yaw_angle, float roll_angle, float pitch_angle, float time_s)
{
    uint8_t seq[2] = {0, 0};
    uint8_t resp[8];
    uint8_t resp_len = 0;

    /* 先发送位置控制命令帧，并保存序列号 */
    if (!rs_send_move_to(seq, yaw_angle, roll_angle, pitch_angle, time_s))
        return RS_ERROR;

    /* 等待该序列号对应的应答帧（0x0E 命令集、0x00 命令 ID） */
    return rs_wait_response(seq, 0x0E, 0x00, resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
}

/* @brief 设置位置控制中各轴是否有效
 * @param axis  轴类型（YAW/ROLL/PITCH）
 * @param valid true=该轴参与位置控制，false=该轴无效
 * @return true=设置成功，false=轴类型无效 */
bool set_axis_valid(enum AxisType axis, bool valid)
{
    switch (axis)
    {
    case YAW:
        /* BIT1：0=Yaw 有效，1=Yaw 无效 */
        valid ? (position_ctrl_byte &= ~BIT1) : (position_ctrl_byte |= BIT1);
        break;
    case ROLL:
        /* BIT2：0=Roll 有效，1=Roll 无效 */
        valid ? (position_ctrl_byte &= ~BIT2) : (position_ctrl_byte |= BIT2);
        break;
    case PITCH:
        /* BIT3：0=Pitch 有效，1=Pitch 无效 */
        valid ? (position_ctrl_byte &= ~BIT3) : (position_ctrl_byte |= BIT3);
        break;
    default:
        return false;
    }
    return true;
}

/* @brief 设置位置控制模式
 * @param type 绝对控制(ABSOLUTE_CONTROL) 或增量控制(INCREMENTAL_CONTROL)
 * @return true */
bool set_move_mode(enum MoveMode type)
{
    /* BIT0=1 绝对位置模式，BIT0=0 增量（相对）位置模式 */
    if (type == ABSOLUTE_CONTROL)
        position_ctrl_byte |= BIT0;   /* 绝对位置模式 */
    else
        position_ctrl_byte &= ~BIT0;  /* 相对位置模式 */
    return true;
}

/* ================= 2.3.4.2 手持云台速度控制 ================= */
bool set_speed(int16_t yaw_speed, int16_t roll_speed, int16_t pitch_speed)
{
    uint8_t data_payload[7];
    uint8_t resp[8];
    uint8_t resp_len = 0;

    if (!(yaw_speed >= 0 && yaw_speed <= 3600 &&
          roll_speed >= 0 && roll_speed <= 3600 &&
          pitch_speed >= 0 && pitch_speed <= 3600))
    {
        rt_kprintf("Error!! DJI_RS_SDK.c Function: set_speed overflow!\n");
        return false;
    }

    data_payload[0] = yaw_speed & 0xFF;
    data_payload[1] = (yaw_speed >> 8) & 0xFF;
    data_payload[2] = roll_speed & 0xFF;
    data_payload[3] = (roll_speed >> 8) & 0xFF;
    data_payload[4] = pitch_speed & 0xFF;
    data_payload[5] = (pitch_speed >> 8) & 0xFF;
    data_payload[6] = speed_ctrl_byte;

    return rs_send_and_wait(0x0E, 0x01, data_payload, sizeof(data_payload),
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT) == EXECUTION_SUCCESSFUL;
}

bool set_speed_mode(enum SpeedControl speed_type, enum FocalControl focal_type)
{
    if (speed_type == sENABLED)
        speed_ctrl_byte |= BIT7;    /* 接手速度控制权 */
    else
        speed_ctrl_byte &= ~BIT7;   /* 释放速度控制权 */

    if (focal_type == fENABLED)
        speed_ctrl_byte &= ~BIT3;   /* 移动速度考虑相机焦距影响 */
    else
        speed_ctrl_byte |= BIT3;    /* 移动速度不考虑相机焦距影响 */
    return true;
}

/* ================= 2.3.4.3 获取手持云台角度信息 ================= */
/* @brief 获取云台角度信息（姿态角或关节角）
 * @param data_type GIMBAL_ATTITUDE_ANGLE(0x01) 或 GIMBAL_JOINT_ANGLE(0x02)
 * @param yaw/roll/pitch 输出角度（0.1°，可传空指针跳过）
 * @return 云台返回码或 RS_TIMEOUT/RS_ERROR */
uint8_t get_gimbal_angle(uint8_t data_type, int16_t *yaw, int16_t *roll, int16_t *pitch)
{
    uint8_t resp[16];
    uint8_t resp_len = 0;
    uint8_t code;

    code = rs_send_and_wait(0x0E, 0x02, &data_type, 1,
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    /* 应答数据至少 8 字节：数据段首字节为角度类型，随后 3 组角度 */
    if (resp_len < 8)
        return RS_ERROR;
    if (resp[1] == 0x00 || resp[1] != data_type) /* 应答类型必须与请求一致 */
        return RS_ERROR;

    /* 按小端解析三个轴的角度（0.1°） */
    if (yaw)   *yaw   = (int16_t)((uint16_t)resp[2] | ((uint16_t)resp[3] << 8));
    if (roll)  *roll  = (int16_t)((uint16_t)resp[4] | ((uint16_t)resp[5] << 8));
    if (pitch) *pitch = (int16_t)((uint16_t)resp[6] | ((uint16_t)resp[7] << 8));
    return code;
}

/* ================= 2.3.4.4/2.3.4.5 限位角度 ================= */
uint8_t set_limit_angle(uint8_t pitch_max, uint8_t pitch_min, uint8_t yaw_max, uint8_t yaw_min, uint8_t roll_max, uint8_t roll_min)
{
    uint8_t payload[7] = {0x01, pitch_max, pitch_min, yaw_max, yaw_min, roll_max, roll_min};
    uint8_t resp[8];
    uint8_t resp_len = 0;

    if (pitch_max > 179 || pitch_min > 179 ||
        yaw_max > 179 || yaw_min > 179 ||
        roll_max > 179 || roll_min > 179)
    {
        rt_kprintf("Error!! set_limit_angle overflow!\n");
        return RS_ERROR;
    }

    return rs_send_and_wait(0x0E, 0x03, payload, sizeof(payload),
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
}

uint8_t get_limit_angle(uint8_t *pitch_max, uint8_t *pitch_min, uint8_t *yaw_max, uint8_t *yaw_min, uint8_t *roll_max, uint8_t *roll_min)
{
    uint8_t ctrl = 0x01;
    uint8_t resp[16];
    uint8_t resp_len = 0;
    uint8_t code;

    code = rs_send_and_wait(0x0E, 0x04, &ctrl, 1,
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 7)
        return RS_ERROR;

    if (pitch_max) *pitch_max = resp[1];
    if (pitch_min) *pitch_min = resp[2];
    if (yaw_max)   *yaw_max   = resp[3];
    if (yaw_min)   *yaw_min   = resp[4];
    if (roll_max)  *roll_max  = resp[5];
    if (roll_min)  *roll_min  = resp[6];
    return code;
}

/* ================= 2.3.4.6/2.3.4.7 电机力度 ================= */
uint8_t set_motor_stiffness(uint8_t pitch, uint8_t roll, uint8_t yaw)
{
    uint8_t payload[4] = {0x01, pitch, roll, yaw};
    uint8_t resp[8];
    uint8_t resp_len = 0;

    if (pitch > 100 || roll > 100 || yaw > 100)
    {
        rt_kprintf("Error!! set_motor_stiffness overflow!\n");
        return RS_ERROR;
    }

    return rs_send_and_wait(0x0E, 0x05, payload, sizeof(payload),
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
}

uint8_t get_motor_stiffness(uint8_t *pitch, uint8_t *yaw, uint8_t *roll)
{
    uint8_t ctrl = 0x01;
    uint8_t resp[16];
    uint8_t resp_len = 0;
    uint8_t code;

    code = rs_send_and_wait(0x0E, 0x06, &ctrl, 1,
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 4)
        return RS_ERROR;

    if (pitch) *pitch = resp[1];
    if (yaw)   *yaw   = resp[2];
    if (roll)  *roll  = resp[3];
    return code;
}

/* ================= 2.3.4.8 参数推送设置 ================= */
uint8_t set_param_push(uint8_t ctrl)
{
    uint8_t resp[8];
    uint8_t resp_len = 0;

    return rs_send_and_wait(0x0E, 0x07, &ctrl, 1,
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
}

/* ================= 2.3.4.10 获取模块版本号 ================= */
uint8_t get_module_version(uint32_t device_id, uint32_t *version)
{
    uint8_t payload[4];
    uint8_t resp[16];
    uint8_t resp_len = 0;
    uint8_t code;

    payload[0] = device_id & 0xFF;
    payload[1] = (device_id >> 8) & 0xFF;
    payload[2] = (device_id >> 16) & 0xFF;
    payload[3] = (device_id >> 24) & 0xFF;

    code = rs_send_and_wait(0x0E, 0x09, payload, sizeof(payload),
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 9)
        return RS_ERROR;

    if (version)
        *version = (uint32_t)resp[5] |
                   ((uint32_t)resp[6] << 8) |
                   ((uint32_t)resp[7] << 16) |
                   ((uint32_t)resp[8] << 24);
    return code;
}

/* ================= 2.3.4.11 外部设备控制命令推送（无应答） ================= */
void joystick_control(int16_t pitch_speed, int16_t roll_speed, int16_t yaw_speed)
{
    uint8_t payload[7];

    payload[0] = 0x01;   /* 摇杆遥控器 */
    payload[1] = pitch_speed & 0xFF;
    payload[2] = (pitch_speed >> 8) & 0xFF;
    payload[3] = roll_speed & 0xFF;
    payload[4] = (roll_speed >> 8) & 0xFF;
    payload[5] = yaw_speed & 0xFF;
    payload[6] = (yaw_speed >> 8) & 0xFF;

    rs_send_cmd(0x0E, 0x0A, payload, sizeof(payload), RT_NULL);
}

void dial_control(int16_t dial_speed)
{
    uint8_t payload[3];

    payload[0] = 0x02;   /* 拨轮遥控器 */
    payload[1] = dial_speed & 0xFF;
    payload[2] = (dial_speed >> 8) & 0xFF;

    rs_send_cmd(0x0E, 0x0A, payload, sizeof(payload), RT_NULL);
}

/* ================= 2.3.4.12/2.3.4.13 用户参数（TLV） ================= */
uint8_t get_user_param(uint8_t *read_ids, uint8_t id_num, uint8_t *tlv_buf, uint8_t *tlv_len)
{
    uint8_t resp[64];
    uint8_t resp_len = 0;
    uint8_t code;

    if (read_ids == RT_NULL || id_num == 0)
        return RS_ERROR;

    code = rs_send_and_wait(0x0E, 0x0B, read_ids, id_num,
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 1)
        return RS_ERROR;

    if (tlv_len)
        *tlv_len = resp_len - 1;
    if (tlv_buf && resp_len > 1)
        rt_memcpy(tlv_buf, &resp[1], resp_len - 1);
    return code;
}

uint8_t set_user_param(uint8_t tlv_id, uint8_t tlv_length, uint8_t *tlv_data)
{
    uint8_t payload[34];
    uint8_t resp[64];
    uint8_t resp_len = 0;

    if (tlv_length > 32)
        return RS_ERROR;
    if (tlv_length > 0 && tlv_data == RT_NULL)
        return RS_ERROR;

    payload[0] = tlv_id;
    payload[1] = tlv_length;
    if (tlv_length > 0)
        rt_memcpy(&payload[2], tlv_data, tlv_length);

    return rs_send_and_wait(0x0E, 0x0C, payload, 2 + tlv_length,
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
}

/* ================= 2.3.4.14/2.3.4.15 工作模式与回中/自拍/跟随 ================= */
uint8_t set_gimbal_work_mode(uint8_t work_mode, uint8_t orientation)
{
    uint8_t payload[2] = {work_mode, orientation};
    uint8_t resp[8];
    uint8_t resp_len = 0;

    return rs_send_and_wait(0x0E, 0x0D, payload, sizeof(payload),
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
}

uint8_t set_gimbal_center_selfie(uint8_t action)
{
    uint8_t payload[2] = {0xFE, action};
    uint8_t resp[8];
    uint8_t resp_len = 0;

    return rs_send_and_wait(0x0E, 0x0E, payload, sizeof(payload),
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
}

uint8_t set_gimbal_follow_mode(uint8_t mode)
{
    uint8_t payload[2] = {mode, 0x00};
    uint8_t resp[8];
    uint8_t resp_len = 0;

    return rs_send_and_wait(0x0E, 0x0E, payload, sizeof(payload),
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
}

/* ================= 2.3.4.16/2.3.4.18 自动校准、智能跟随 ================= */
uint8_t set_auto_calibration(uint8_t enable, uint8_t type)
{
    uint8_t payload[3] = {0x00, 0x01, (uint8_t)((type << 1) | (enable & 0x01))};

    return rs_send_cmd(0x0E, 0x0F, payload, sizeof(payload), RT_NULL) ?
           EXECUTION_SUCCESSFUL : RS_ERROR;
}

uint8_t set_smart_follow(uint8_t cmd)
{
    if (cmd != 0x03)
    {
        rt_kprintf("Error!! set_smart_follow invalid cmd: %d\n", cmd);
        return RS_ERROR;
    }

    return rs_send_cmd(0x0E, 0x11, &cmd, 1, RT_NULL) ?
           EXECUTION_SUCCESSFUL : RS_ERROR;
}

/* ================= 2.3.4.19 跟焦器电机控制 ================= */
void focus_motor_position_control(uint16_t position)
{
    uint8_t payload[5] = {0x01, 0x00, 0x02,
                          position & 0xFF, (position >> 8) & 0xFF};

    rs_send_cmd(0x0E, 0x12, payload, sizeof(payload), RT_NULL);
}

uint8_t focus_motor_calibrate(uint8_t motor_type, uint8_t cmd)
{
    uint8_t payload[3] = {0x02, motor_type, cmd};
    uint8_t resp[16];
    uint8_t resp_len = 0;

    return rs_send_and_wait(0x0E, 0x12, payload, sizeof(payload),
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
}

uint8_t focus_motor_get_position(uint8_t motor_type, uint8_t *calib_state, uint32_t *position)
{
    uint8_t payload[2] = {0x15, motor_type};
    uint8_t resp[16];
    uint8_t resp_len = 0;
    uint8_t code;

    code = rs_send_and_wait(0x0E, 0x12, payload, sizeof(payload),
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 8)
        return RS_ERROR;

    if (calib_state)
        *calib_state = resp[3];
    if (position)
        *position = (uint32_t)resp[4] |
                    ((uint32_t)resp[5] << 8) |
                    ((uint32_t)resp[6] << 16) |
                    ((uint32_t)resp[7] << 24);
    return code;
}

/* ================= 2.3.5 相机命令集 ================= */
uint8_t camera_action(uint16_t action)
{
    uint8_t payload[2] = {action & 0xFF, (action >> 8) & 0xFF};
    uint8_t resp[8];
    uint8_t resp_len = 0;

    return rs_send_and_wait(0x0D, 0x00, payload, sizeof(payload),
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
}

uint8_t camera_get_status(uint8_t *status)
{
    uint8_t ctrl = 0x01;
    uint8_t resp[8];
    uint8_t resp_len = 0;
    uint8_t code;

    code = rs_send_and_wait(0x0D, 0x01, &ctrl, 1,
                            resp, sizeof(resp), &resp_len, RS_RESPONSE_TIMEOUT);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 2)
        return RS_ERROR;

    if (status)
        *status = resp[1];
    return code;
}

/* ================= 连接/断开 ================= */
/* @brief 连接云台：通过查询 0x00000001 模块版本确认链路
 * @return true=收到成功应答，false=超时/失败 */
bool rs_connect(void)
{
    uint32_t version = 0;
    return get_module_version(0x00000001, &version) == EXECUTION_SUCCESSFUL;
}

/* @brief 断开云台：发送参数推送禁用命令（0x02=禁用）
 * @return true=云台返回成功 */
bool rs_disconnect(void)
{
    return set_param_push(0x02) == EXECUTION_SUCCESSFUL;
}
