#include "DJI_RS_SDK.h"

#define RS_RESPONSE_TIMEOUT 2000




// 初始化函数
void DJIRonin() {
    position_ctrl_byte = 0x00;  // 初始化为绝对模式
    position_ctrl_byte |= BIT1; // 默认绝对位置控制
    speed_ctrl_byte = 0x00;     // 初始禁用速度控制
    speed_ctrl_byte |= BIT3 ;    // 默认禁用焦距影响

    Enc_Set(NoEnc);
    CmdType_Set(MustResponse, CommandFrame);



}


/**
 * @brief move_to 手持云台位置控制（文档第五页，2.3.4.1章节）
 * @param yaw_angle   航向轴角度，精度0.1°（范围 -180° ~ +180°）
 * @param roll_angle  横滚轴角度，精度0.1°（范围 -180° ~ +180°）
 * @param pitch_angle 俯仰轴角度，精度0.1°（范围 -90° ~ +90°）
 * @param time_s      执行速度,   精度0.1秒。设置手持云台运动速度
 * @return 应答帧返回码：
 *         EXECUTION_SUCCESSFUL = 0,执行成功
 *         PARSE_ERROR = 1 解析错误
 *         EXECUTION_FAILS = 2执行失败
 *         UNDEFINED_ERROR = 0xFF 未定义错误
 *         RS_TIMEOUT = 3   超时
 *         RS_ERROR   = 4   moveto函数执行失败
 */
uint8_t move_to(float yaw_angle, float roll_angle, float pitch_angle, float time_s){
    RS_Msg msg;
    rt_err_t res;
    uint8_t Seq[2] = {0}; // 初始化SEQ，避免随机值
    uint8_t ret_code = 0xFF; // 默认返回错误码
    rt_tick_t start_tick = rt_tick_get(); // 记录循环开始时间

    // 第一步：校验moveto调用是否成功（指令是否发送出去）
    if (!moveto(Seq, yaw_angle, roll_angle, pitch_angle, time_s)) {
        rt_kprintf("move_to: moveto call failed (param error/send failed)\n");
        return RS_ERROR; // 返回自定义错误码
    }


    // 第二步：带超时的应答帧匹配循环
    while (1) {
        // 检查是否超时（超过5秒则退出）
        if (rt_tick_get() - start_tick > RT_TICK_PER_SECOND * (RS_RESPONSE_TIMEOUT/1000)) {
            rt_kprintf("move_to: response timeout (SEQ: %02X%02X)\n", Seq[0], Seq[1]);
            return RS_TIMEOUT; // 超时错误码
        }

        // 非阻塞接收（每次等待100ms，避免永久阻塞）
        res = rt_mq_recv(rs_res_mq, &msg, sizeof(RS_Msg), 100);

        // 仅处理接收成功的情况
        if ((res != RT_ERROR)||(res != RT_ETIMEOUT)) {
            // 匹配目标SEQ
            if (msg.seq[0] == Seq[0] && msg.seq[1] == Seq[1]) {
                ret_code = msg.data[0];
                //rt_kprintf("move_to: success, return code: %02X (SEQ: %02X%02X)\n", ret_code, Seq[0], Seq[1]);
                return ret_code;
            } else {
                // 非目标SEQ，继续循环
                //rt_kprintf("move_to: ignore unmatched SEQ: %02X%02X\n", Seq[0], msg.seq[1]);
            }
        }
        // 处理接收错误/超时（继续循环，直到总超时）
        else if (res == RT_ETIMEOUT) {
            continue;
        }
        else {
            rt_kprintf("move_to: mq recv error: %d\n", res);
            return RS_ERROR;
        }
    }


}


// 轴方向反转设置 (文档2.3.4.1)
bool set_inverted_axis(enum AxisType axis, bool invert) {
    switch (axis) {
        case YAW:
            invert ? (position_ctrl_byte |= BIT2) : (position_ctrl_byte &= ~BIT2);
            break;
        case ROLL:
            invert ? (position_ctrl_byte |= BIT3) : (position_ctrl_byte &= ~BIT3);
            break;
        case PITCH:
            invert ? (position_ctrl_byte |= BIT4) : (position_ctrl_byte &= ~BIT4);
            break;
        default:
            return false;
    }
    return true;
}

// 运动模式设置 (文档2.3.4.1)
bool set_move_mode(enum MoveMode type) {
    if (type == ABSOLUTE_CONTROL) {
        position_ctrl_byte |= BIT1;   // 绝对位置模式
    } else {
        position_ctrl_byte &= ~BIT1;  // 相对位置模式
    }
    return true;
}

// 速度控制指令 (文档2.3.4.2)
bool set_speed(int16_t yaw_speed, int16_t roll_speed, int16_t pitch_speed) {
    // 速度范围校验 (0.1°/s)
    if (!(yaw_speed >= 0 && yaw_speed <= 3600 &&
          roll_speed >= 0 && roll_speed <= 3600 &&
          pitch_speed >= 0 && pitch_speed <= 3600)) {
        rt_kprintf("Error!! DJI_RS_SDK.c Function: set_speed overflow!\n");
        return false;
    }

    // 数据载荷 (小端模式)
    uint8_t data_payload[] = {
        yaw_speed & 0xFF,   (yaw_speed >> 8) & 0xFF,   // Yaw速度
        roll_speed & 0xFF,  (roll_speed >> 8) & 0xFF,  // Roll速度
        pitch_speed & 0xFF, (pitch_speed >> 8) & 0xFF, // Pitch速度
        speed_ctrl_byte     // 速度控制标志位
    };

    // 组合指令帧并发送
    uint8_t* cmd = Combine(0x03, 0x0E, 0x01, data_payload, sizeof(data_payload));
    if (!cmd) return false;

    bool ret = send_data(cmd, cmd[1]);
    rt_free(cmd);
    return ret;
}

// 速度模式设置 (文档2.3.4.2)
bool set_speed_mode(enum SpeedControl speed_type, enum FocalControl focal_type) {
    // 速度控制使能
    if (speed_type == sENABLED) {
        speed_ctrl_byte |= BIT7;
    } else {
        speed_ctrl_byte &= ~BIT7;
    }

    // 焦距影响使能
    if (focal_type == fENABLED) {
        speed_ctrl_byte &= ~BIT3;
    } else {
        speed_ctrl_byte |= BIT3;
    }
    return true;
}

