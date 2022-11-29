# DJI-Ronin-SDK

#### Overview

The source of DJI RS SDK to control DJI RS2/RS3 Pro stabilizer on STM32 with HAL library / RT-Thread RTOS, use can bus, rewrite with C according to https://github.com/ConstantRobotics/DJIR_SDK 
（prototype has written by C++）. 

在用 HAL库/RT-Thread 平台的STM32上控制大疆RS2/RS3 Pro 稳定器的DJI RS SDK源代码，用C重新实现参考项目中用C++的实现



#### Instructions for use HAL version

1.  Create a stm32 project, enable can1 and setting baud rate on 1M in STM32CubeMX
  
    在STM32CubeMX里创建工程，使能can1并设置1M的波特率
    
2.  Add HAL version source, #include"DJI_RS_SDK.h" at you want use it. The parameters of can bus have been configured，needn't config can1.
   
    加此项目HAL版本的源代码，在需要的地方 #include"DJI_RS_SDK.h",can总线已经配置好不需要另外配置can1。

3.  Use source: DJI_Ronin() at main() to init ，and then can use source of DJI_RS_SDK.h ，for example Moveto();
   
    首先需要在main()中使用DJI_Ronin()初始化，之后就可以使用DJI_RS_SDK.h中定义的代码例如Moveto()


#### Instructions for use RT-Thread Version

1.  Add RTThread version source, open CAN device in RT Thread（in this is can1)，#include"DJI_RS_SDK.h" at you need
   
    添加RT Thread版本的源代码，打开RTT的CAN设备（这里默认can1），在需要地方#include"DJI_RS_SDK.h"
    
2.  Use source: DJI_Ronin() in board auto init like Button_Init() in Ronin.c
   
    在 板级自动初始化（ INIT_BOARD_EXPORT() ）中引用DJI Ronin()，就像Ronin.c中的Button_Init()一样
    
3.  Create a thread, to send CAN frame. can1 device has already been auto opened in Handle.c. So **DON'T OPEN CAN1 AGAIN!!!**
  
    创建一个线程，发送CAN数据。设备 can1已经在Handle.c中自动打开，**不要第二次打开CAN1！！**


**PS： 因为手上只有如影S而且如影S并不支持DJI RS SDK，目前尚不能确认can总线上所发的数据是否正确。**
