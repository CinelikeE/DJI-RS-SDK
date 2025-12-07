# DJI-Ronin-SDK For RT Thread Ver1.0

#### Overview


在用RT-Thread OS上控制大疆RS2/RS3 Pro 稳定器的DJI RS SDK源代码，用C重新实现参考项目https://github.com/ConstantRobotics/DJIR_SDK用C++的实现

Ver1.0 版本：修改DJI_RS_SDK.c中的实现，优化Combine.c中逻辑

开发板为RT Thread官方开发板HMI-Board，mcu为瑞萨Renesas RA6M3 ，RTT中打开HW can filter时can总线无法打开，之前版本使用stm32时可以正常打开

待验证是否能够控制DJI RS2，并由此完善API

#### Instructions for use RT-Thread Version

1.     添加RT Thread版本的源代码，打开RTT的CAN设备，在需要地方#include"DJI_RS_SDK.h"
    
2.     在板级自动初始化（ INIT_BOARD_EXPORT() ）中引用DJI Ronin()，就像Ronin.c中的Button_Init()一样
    
3.     创建一个线程，发送CAN数据。设备 can1已经在Handle.c中自动打开，**不要第二次打开同一条CAN！！**


