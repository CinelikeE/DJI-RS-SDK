# DJI-RS-SDK 

#### Overview



这是一个保留了STM32HAL Ver0 版本，暂且留着但很久都应该不会再开发了

保留HAL版本Ver0原因：

1、当时写HAL版本的时候发现can邮箱一次只能缓冲约3帧can数据，需要进行延时等待can邮箱清空

2、RTT接管了内存管理，由于使用了大量指针并且频繁malloc free，rtt中函数为rt_malloc()和rt_free()，懒得改×1

3、RTT进一步封装了mcu外设以实现跨平台甚至跨架构兼容性，有点忘了stm32上HAL怎么弄can总线，懒得改×2

ps3、（RT Thread的兼容性似乎对cortexM和riscV较好，但个人体验是似乎只对stm32比较完善，同样是cortex M4的RA6M3在can总线上体验就没那么好）

4、留着，如果想搞HAL开发的话可以当参考，但具体实现要跟随更新版本，或者等我有一天兴致大发去更新



