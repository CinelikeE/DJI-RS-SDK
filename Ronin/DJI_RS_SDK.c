#include "DJI_RS_SDK.h"

uint8_t position_ctrl_byte  = 0x00;  // 位置控制标志位
uint8_t speed_ctrl_byte     = 0x00;  // 速度控制标志位


// 初始化函数
void DJIRonin() {
    position_ctrl_byte = 0x00;  // 初始化为绝对模式
    position_ctrl_byte |= BIT1; // 默认绝对位置控制
    speed_ctrl_byte = 0x00;     // 初始禁用速度控制
    speed_ctrl_byte |= BIT3;    // 默认禁用焦距影响
}
int DJIRonin_Init(){
    rt_thread_t tid3 = RT_NULL;

        tid3 = rt_thread_create("DJIRonin_Init", DJIRonin, RT_NULL, 1024, 11, 10);
    if(tid3 != RT_NULL)
        rt_thread_startup(tid3);

    return RT_EOK;
}INIT_APP_EXPORT(DJIRonin_Init);

bool move_to(float yaw_angle, float roll_angle, float pitch_angle, float time_s){

    int16_t yaw = (int16_t)(yaw_angle * 10);
    int16_t roll = (int16_t)(roll_angle * 10);
    int16_t pitch = (int16_t)(pitch_angle * 10);
    uint8_t time = (uint8_t)(time_s * 10);

    if (!(yaw >= -1800 && yaw <= 1800 &&
          roll >= -300 && roll <= 300 &&  // 横滚限制更严格
          pitch >= -560 && pitch <= 1460 &&  // 俯仰范围
          time >= 1)) {  // 最小时间0.1s
        rt_kprintf("Error!! DJI_RS_SDK.c Function: moveto overflow!\n");
        return false;
    }

    // 数据载荷 (小端模式)
    uint8_t data_payload[] = {
        yaw & 0xFF,        (yaw >> 8) & 0xFF,   // Yaw：低字节→高字节（小端）
        roll & 0xFF,       (roll >> 8) & 0xFF,  // Roll：低字节→高字节（小端）
        pitch & 0xFF,      (pitch >> 8) & 0xFF, // Pitch：低字节→高字节（小端）
        position_ctrl_byte,                     // 控制标志位
        time                                    // 执行时间
    };

    uint8_t* cmd = Combine(0x03, 0x0E, 0x00, data_payload, sizeof(data_payload));
    if (!cmd) return false;

    bool ret = send_data(cmd, cmd[1]);  // cmd[1]是帧长度
    rt_free(cmd);
    return ret;

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

