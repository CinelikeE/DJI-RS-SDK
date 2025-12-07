# DJI-Ronin-SDK For RT Thread Ver1.0

#### Overview


在用RT-Thread OS上控制大疆RS2/RS3 Pro 稳定器的DJI RS SDK源代码，用C重新实现参考项目https://github.com/ConstantRobotics/DJIR_SDK用C++的实现

Ver1.0 版本：修改DJI_RS_SDK.c中的实现，优化Combine.c中逻辑

开发板为RT Thread官方开发板HMI-Board，mcu为瑞萨Renesas RA6M3 ，RTT中打开HW can filter时can总线无法打开，之前版本使用stm32时可以正常打开

待验证是否能够控制DJI RS2，若成功将继续完善API（写完代码验证发出字符后发现手上没有杜邦线）

删除了Ver0中的RTT版本，保留STM32HAL Ver0 版本
保留原因：1、当时写HAL版本的时候发现can邮箱一次只能缓冲约3帧can数据，需要进行延时等待can邮箱清空
         2、RTT接管了内存管理，由于使用了大量指针并且频繁malloc free，rtt中函数为rt_malloc()和rt_free()，懒得改×1
         3、RTT进一步封装了mcu外设以实现跨平台甚至跨架构兼容性，有点忘了stm32上HAL怎么弄can总线，懒得改×2
         ps3、（RT Thread的兼容性似乎对cortexM和riscV较好，但个人体验是似乎只对stm32比较完善，同样是cortex M4的RA6M3在can总线上体验就没那么好）
         4、留着，如果想搞HAL开发的话可以当参考，但具体实现要跟随更新版本，或者等我有一天兴致大发去更新

#### Instructions for use RT-Thread Version

1. 添加RT Thread版本的源代码，打开RTT的CAN设备，在需要地方#include"DJI_RS_SDK.h"
    
2. 引用DJI Ronin()，但目前来说这个函数似乎没有用。。。
    
3. 创建一个线程，发送CAN数据。设备 can1已经在Handle.c中自动打开，**不要第二次打开同一条CAN！！**


