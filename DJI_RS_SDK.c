#include "DJI_RS_SDK.h"

uint8_t position_ctrl_byte = 0;
uint8_t speed_ctrl_byte  = 0;

void DJIRonin(void){
    position_ctrl_byte = 0;
    speed_ctrl_byte = 0;

    position_ctrl_byte |= BIT1;
    speed_ctrl_byte |= BIT3;
}

bool move_to(float yaw_angle, float roll_angle, float pitch_angle, float time_s){
    uint8_t *CMD;
    uint8_t cmd_type = 0x03;
    uint8_t cmd_set = 0x0E;
    uint8_t cmd_id = 0x00;
    int16_t yaw = (int16_t)(yaw_angle * 10);
    int16_t roll = (int16_t)(roll_angle * 10);
    int16_t pitch = (int16_t)(pitch_angle * 10);
    uint8_t time = (uint8_t)(time_s * 10);

    if( (18000>= yaw) && (yaw >= -18000) && (3000 >= roll) && (roll >= -3000) && (1460 >= pitch) && (pitch >= -560) )
    {
        uint8_t data_payload[]={
                ((uint8_t*)&yaw)[0],((uint8_t*)&yaw)[1],
                ((uint8_t*)&roll)[0],((uint8_t*)&roll)[1],
                ((uint8_t*)&pitch)[0],((uint8_t*)&pitch)[1],
                position_ctrl_byte, time
        };

        CMD = Combine(cmd_type,cmd_set,cmd_id,data_payload, sizeof(data_payload));
        uint8_t CombineCMD[CMD[1]];
        for (int i = 0; i < CMD[1]; i++) {
            CombineCMD[i] = CMD[i];
        }
        free(CMD);

        for (int i = 0; i < sizeof(CombineCMD); i++) {
            printf("%x\t",CombineCMD[i]);
        }

        if( true )
            return true;
    }
    else
        return false;
}


bool set_inverted_axis(enum AxisType axis, bool invert){
    if(axis == YAW){
        if(invert)
            position_ctrl_byte |= BIT2;
        else
            position_ctrl_byte &= ~BIT2;
    }

    if(axis == ROLL){
        if(invert)
            position_ctrl_byte |= BIT3;
        else
            position_ctrl_byte &= ~BIT3;
    }

    if(axis == PITCH){
        if(invert)
            position_ctrl_byte |= BIT4;
        else
            position_ctrl_byte &= ~BIT4;
    }

    return true;
}


bool set_move_mode(enum MoveMode type){
    if(type == INCREMENTAL_CONTROL)
        position_ctrl_byte &= ~BIT2;
    else
        position_ctrl_byte |= BIT1;

    return true;
}

bool set_speed(int16_t yaw, int16_t roll, int16_t pitch){
    uint8_t *CMD;
    uint8_t cmd_type = 0x03;
    uint8_t cmd_set = 0x0E;
    uint8_t cmd_id = 0x01;
    uint8_t data_payload[]={
            ((uint8_t*)&yaw)[0],((uint8_t*)&yaw)[1],
            ((uint8_t*)&roll)[0],((uint8_t*)&roll)[1],
            ((uint8_t*)&pitch)[0],((uint8_t*)&pitch)[1],
            speed_ctrl_byte
    };

    CMD = Combine(cmd_type,cmd_set,cmd_id,data_payload, sizeof(data_payload));

    uint8_t CombineCMD[CMD[1]];
    for (int i = 0; i < CMD[1]; i++) {
        CombineCMD[i] = CMD[i];
    }
    free(CMD);

    for (int i = 0; i < sizeof(CombineCMD); i++) {
        printf("%x\t",CombineCMD[i]);
    }

    if( 1)
        return true;
    else
        return false;
}

bool set_speed_mode(enum SpeedControl speed_type, enum FocalControl focal_type){
    if(speed_type == sENABLED) {
        speed_ctrl_byte |= BIT7;
    }else
        speed_ctrl_byte &= ~BIT7;

    if(focal_type == fENABLED){
        speed_ctrl_byte &= ~BIT3;
    } else
        speed_ctrl_byte |= BIT3;

    return true;
}



