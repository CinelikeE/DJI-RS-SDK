#include "Handle.h"

uint32_t *box;
CAN_TxHeaderTypeDef canTX ;

void CanTX_Init(){

    canTX.StdId = 0x222;
    canTX.ExtId = 0;
    canTX.IDE   = CAN_ID_STD;
    canTX.RTR   = CAN_RTR_DATA;
}

uint32_t send_data(uint8_t *data, uint8_t data_len){


    uint32_t frame_num;                                     //总帧数
    uint32_t full_frame_num = data_len / 8;                 //满数据帧数
    uint32_t left_frame_len = data_len % 8;                 //剩余数据长度


    if (left_frame_len == 0)
        frame_num = full_frame_num;
    else
        frame_num = full_frame_num + 1;                     //frame number judge 总帧数判断

    uint32_t data_offset   = 0;                             //数据计数位
    uint32_t frame_len     = 0;                             //单帧长度
    uint32_t left_data_len = data_len;                      //剩余数据长度
    uint32_t count_buf = 0;
    for (int i = 0; i < frame_num; i++) {

        if (left_data_len >= 8)
            frame_len = 8;
        else
            frame_len = left_data_len;                      //single frame length judge 单帧长度判断

        uint8_t *buf = (uint8_t*)malloc(frame_len);    //single frame send data 单帧发送数据

        for (int j = 0; j < frame_len; j++) {

            buf[j] = data[data_offset + j];


        }
        uint8_t *send_buf = buf;

/** here is how to send data frame with can in stm32 with different method **********************************************/

        canTX.DLC  = frame_len;


        HAL_CAN_AddTxMessage(&hcan1,&canTX,send_buf,box);

        count_buf ++;

        if (count_buf == 3)
            HAL_Delay(2);


/************************************************************************************************************************/
        free(send_buf);
        left_data_len -= 8;                                 //left data length = last left data length - 8  剩余数据 = 当前剩余数据 - 8
        data_offset += 8;                                   //
    }

    return 1;

}