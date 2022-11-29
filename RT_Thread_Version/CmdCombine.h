#ifndef CMDCOMBINE_H
#define CMDCOMBINE_H
#include "DJI_RS_SDK.h"
#include "custom_crc16.h"
#include "custom_crc32.h"

uint8_t *Combine(uint8_t cmd_type, uint8_t cmd_set, uint8_t cmd_id, uint8_t *data, uint8_t data_lenth);

uint8_t  *seq_num();

#endif //CMDCOMBINE_H
