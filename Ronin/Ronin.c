#include "rtthread.h"
#include "DJI_RS_SDK.h"
#include "board.h"



float yaw   = 0;
float pitch = 0;





void LocationCtrl(){

    rt_thread_mdelay(1000);

    while (1)
    {
        if( move_to(120, 0, 0, 2.0f) == EXECUTION_SUCCESSFUL )

        rt_thread_delay(5000);
        if( move_to(-120, 0, 0, 2.0f) == EXECUTION_SUCCESSFUL )
        rt_thread_delay(5000);
        rt_kprintf("\r\n");
    }

}

int CanT_Init(){
    rt_thread_t tid3 = RT_NULL;

    rt_thread_delay(1000);
    tid3 = rt_thread_create("LocationCtrl", LocationCtrl, RT_NULL, 1024, 11, 10);
    if(tid3 != RT_NULL)
        rt_thread_startup(tid3);

    return RT_EOK;
}INIT_APP_EXPORT(CanT_Init);




