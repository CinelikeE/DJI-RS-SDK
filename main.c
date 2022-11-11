#include "DJI_RS_SDK.h"


int main() {
    DJIRonin();
    move_to(3.2, 4.8, 6.4, 2);
    printf("\n");
    set_speed(5,5,5);
    printf("\n");




    return 0;
}

