#include "DJI_RS_Set.h"


void Enc_Set(uint8_t Encode){
    enc = Encode;
}

void CmdType_Set(uint8_t Response, uint8_t Type){
    CmdType = (Response & 0x1F) | ((Type << 5) & 0x20);

    if (Response == NoResponse ) {

    }
}




