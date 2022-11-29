#include "rtthread.h"
#include "DJI_RS_SDK.h"
#include "board.h"

#define UP_PIN    GET_PIN(A, 0)
#define DOWN_PIN  GET_PIN(E, 3)
#define LEFT_PIN  GET_PIN(E, 2)
#define RIGHT_PIN GET_PIN(E, 4)

#define LED1_PIN  GET_PIN(F, 10)

float yaw   = 0;
float pitch = 0;

int Button_Init(){
    rt_pin_mode(UP_PIN ,    PIN_MODE_INPUT_PULLDOWN);
    rt_pin_mode(DOWN_PIN ,  PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(LEFT_PIN ,  PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(RIGHT_PIN , PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(LED1_PIN,  PIN_MODE_OUTPUT);
    rt_pin_write(LED1_PIN, PIN_HIGH);

    DJIRonin();
    return RT_EOK;
}INIT_BOARD_EXPORT(Button_Init);



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

void CanTx(){
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1)
    {
        if( move_to(yaw, 0, pitch, 0.5) ){}
        rt_thread_mdelay(500);
    }
#pragma clang diagnostic pop
}

int CanT_Init(){
    rt_thread_t tid3 = RT_NULL;

    tid3 = rt_thread_create("CanTX", CanTx, RT_NULL, 1024, 11, 10);
    if(tid3 != RT_NULL)
        rt_thread_startup(tid3);

    return RT_EOK;
}INIT_APP_EXPORT(CanT_Init);




