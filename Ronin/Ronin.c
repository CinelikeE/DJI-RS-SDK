#include "rtthread.h"
#include "DJI_RS_SDK.h"
#include "board.h"



float yaw   = 0;
float pitch = 0;



/**
void Yaw(){
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
        if ((rt_pin_read(UP_PIN) == PIN_HIGH) && (rt_pin_read(DOWN_PIN) == PIN_HIGH)) {
            pitch += 1;
            if(pitch > 146)
                pitch = 146;

            rt_pin_write(LED1_PIN, PIN_LOW);
            rt_thread_mdelay(100);
            rt_pin_write(LED1_PIN, PIN_HIGH);
        } else if ((rt_pin_read(UP_PIN) == PIN_LOW) && (rt_pin_read(DOWN_PIN) == PIN_LOW)) {
            pitch -= 1;
            if(pitch < -56)
                pitch = -56;

            rt_pin_write(LED1_PIN, PIN_LOW);
            rt_thread_mdelay(100);
            rt_pin_write(LED1_PIN, PIN_HIGH);
        }
        rt_thread_mdelay(10);
    }
#pragma clang diagnostic pop
}

void Pitch(){
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
        if ((rt_pin_read(LEFT_PIN) == PIN_LOW) && (rt_pin_read(RIGHT_PIN) == PIN_HIGH)) {
            yaw -= 1;
            if(yaw < -180)
                yaw = -180;

            rt_pin_write(LED1_PIN, PIN_LOW);
            rt_thread_mdelay(100);
            rt_pin_write(LED1_PIN, PIN_HIGH);
        } else if ((rt_pin_read(LEFT_PIN) == PIN_HIGH) && (rt_pin_read(RIGHT_PIN) == PIN_LOW)) {
            yaw += 1;
            if(yaw > 180)
                yaw = 180;

            rt_pin_write(LED1_PIN, PIN_LOW);
            rt_thread_mdelay(100);
            rt_pin_write(LED1_PIN, PIN_HIGH);
        }
        rt_thread_mdelay(10);
    }
#pragma clang diagnostic pop
}



int Button(){
     rt_thread_t tid1 = RT_NULL;
     rt_thread_t tid2 = RT_NULL;

     tid1 = rt_thread_create("Yaw", Yaw, RT_NULL, 350, 10, 10);
     if (tid1 != RT_NULL)
         rt_thread_startup(tid1);

     tid2 = rt_thread_create("Pitch", Pitch, RT_NULL, 350, 10, 10);
     if (tid2 != RT_NULL)
        rt_thread_startup(tid2);

     return RT_EOK;
}INIT_APP_EXPORT(Button);
**/




void CanTx(){
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    rt_thread_mdelay(1000);
    while (1)
    {
        if( move_to(3.2f, 4.8f, 6.4f, 2.0f) )
        rt_thread_delay(10000);
        rt_kprintf("\r\n");
    }
#pragma clang diagnostic pop
}

int CanT_Init(){
    rt_thread_t tid3 = RT_NULL;

    rt_thread_delay(1000);
    tid3 = rt_thread_create("CanTX", CanTx, RT_NULL, 1024, 11, 10);
    if(tid3 != RT_NULL)
        rt_thread_startup(tid3);

    return RT_EOK;
}INIT_APP_EXPORT(CanT_Init);




