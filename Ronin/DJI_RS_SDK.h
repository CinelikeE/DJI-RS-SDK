#ifndef DJI_RS_SDK_H
#define DJI_RS_SDK_H
#include "rtthread.h"
#include "CmdCombine.h"
#include "Handle.h"
#include "DJI_RS_Set.h"
#include "FrameTransmit.h"
#include "CmdParse.h"


#include "stdbool.h"
#include "string.h"

enum ErrorCode {
    RS_TIMEOUT = 0x03,
    RS_ERROR   = 0x04
};



uint8_t DJI_RS_SDK_Ver;









enum  FLAG  {
    BIT1 = 0x01,
    BIT2 = 0x02,
    BIT3 = 0x04,
    BIT4 = 0x08,
    BIT5 = 0x10,
    BIT6 = 0x20,
    BIT7 = 0x40
};




enum AxisType {
        YAW = 0,
        ROLL = 1,
        PITCH = 2
};

enum MoveMode {
        INCREMENTAL_CONTROL = 0,
        ABSOLUTE_CONTROL = 1
};

enum SpeedControl {
        sDISABLED = 0,
        sENABLED = 1
};

enum FocalControl {
        fENABLED = 0,
        fDISABLED = 1
};

void DJIRonin();

/**
     * @brief connect - Connect to DJI Ronin device
     * @return True if success
     */
bool connect();
/**
 * @brief disconnect - Disconnect from DJI Ronin device
 * @return True if success
 */
bool disconnect();


uint8_t move_to(float yaw_angle, float roll_angle, float pitch_angle, float time_s);

/**
 * @brief set_inverted_axis - Handheld Gimbal Position Control (p.5, 2.3.4.1)
 * @param axis Type of axis (YAW, ROLL, PITCH)
 * @param invert True if invert
 * @return True if success
 */
bool set_inverted_axis(enum AxisType axis, bool invert);

/**
 * @brief set_move_mode Set move mode
 * @param type INCREMENTAL or ABSOLUTE
 * @return True if success
 */
bool set_move_mode(enum MoveMode type);

/**
 * @brief set_speed - Handheld Gimbal Speed Control (p.6, 2.3.4.2)
 * @details This command can only control for 0.5s each time it is issued
 * due to safety reasons. If users require continuous speed, they can send
 * this command periodically. If users want to stop the rotation of three
 * axes immediately, they can set the fields of yaw, pitch, and roll in
 * set_speed_mode method as 0.
 * @param yaw Unit: 0.1°/s (range: 0°/s to 360°/s)
 * @param roll Unit: 0.1°/s (range: 0°/s to 360°/s)
 * @param pitch Unit: 0.1°/s (range: 0°/s to 360°/s)
 * @return True if success
 */
bool set_speed(int16_t yaw_speed, int16_t roll_speed, int16_t pitch_speed);

/**
 * @brief set_speed_mode - Handheld Gimbal Speed Control (p.6, 2.3.4.2)
 * @param speed_type Enable or Disable speed control
 * @param focal_type Enable or Disable the impact of camera focal length
 * into consideration
 * @return True if success
 */
bool set_speed_mode(enum SpeedControl speed_type, enum FocalControl focal_type);


/**
 * @brief get_current_position - Get Gimbal Information (p.6, 2.3.4.3)
 * @param yaw Yaw axis angle (unit: 0.1°)
 * @param roll Roll axis angle (unit: 0.1°)
 * @param pitch Pitch axis angle (unit: 0.1°)
 * @return True if success
 */
// bool get_current_position(int16_t& yaw, int16_t& roll, int16_t& pitch);


#endif //DJI_RS_SDK_H
