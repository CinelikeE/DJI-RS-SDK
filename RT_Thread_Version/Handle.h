#ifndef HANDLE_H
#define HANDLE_H
#include "DJI_RS_SDK.h"
#include "stdint.h"

#include "rtthread.h"
#include "rtdevice.h"
#include "board.h"




int CanTX_Init();
uint32_t send_data(uint8_t *data, uint8_t data_len);

#endif //HANDLE_H
