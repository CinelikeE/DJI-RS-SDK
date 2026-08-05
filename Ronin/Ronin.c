/**
 * @file Ronin.c
 * @brief DJI RS 云台应用示例
 *
 * 演示 RS SDK 的基本用法：
 *  1. 初始化 SDK 协议配置（dji_rs_sdk_init）；
 *  2. 启动 location_ctrl 示例线程：循环让云台在 Yaw 方向
 *     正负 120° 之间往返运动，并周期读取云台姿态角打印；
 *  3. 通过 RT-Thread INIT_APP_EXPORT 自动启动。
 */
#include "rtthread.h"
#include "DJI_RS_SDK.h"
#include "board.h"


/* 查询角度类型：姿态角 */
static uint8_t datatype = GIMBAL_ATTITUDE_ANGLE;

/* 角度查询结果（单位为 0.1°） */
static int16_t yaw   = 0;
static int16_t roll  = 0;
static int16_t pitch = 0;




/* @brief 位置控制示例线程
 *
 * 循环执行：移动到 (120°, 0, 0) 等待 5 秒 -> 读取并打印角度
 *         -> 移动到 (-120°, 0, 0) 等待 5 秒 -> 读取并打印角度 */
static void location_ctrl(void *parameter)
{

    /* 启动后先延时 1 秒，等待 SDK 各模块就绪 */
    rt_thread_mdelay(1000);

    while (1)
    {
        /* 控制云台移动到 Yaw=120°，耗时 2 秒；成功则保持 5 秒 */
        if (move_to(120, 0, 0, 2.0f) == EXECUTION_SUCCESSFUL)
        {
            rt_thread_mdelay(5000);
        }

        /* 读取云台姿态角并打印 */
        if (get_gimbal_angle(datatype, &yaw, &roll, &pitch) == EXECUTION_SUCCESSFUL)
        {
            rt_kprintf("Type: %d, yaw: %d, roll: %d, pitch:%d\r\n", datatype, yaw, roll, pitch);
        }
        else
        {
            rt_kprintf("get_gimbal_angle failed\r\n");
        }

        /* 控制云台移动到 Yaw=-120°，耗时 2 秒；成功则保持 5 秒 */
        if (move_to(-120, 0, 0, 2.0f) == EXECUTION_SUCCESSFUL)
        {
            rt_thread_mdelay(5000);
        }

        rt_kprintf("\r\n");
        /* 再次读取姿态角并打印 */
        if (get_gimbal_angle(datatype, &yaw, &roll, &pitch) == EXECUTION_SUCCESSFUL)
        {
            rt_kprintf("Type: %d, yaw: %d, roll: %d, pitch:%d\r\n", datatype, yaw, roll, pitch);
        }
        else
        {
            rt_kprintf("get_gimbal_angle failed\r\n");
        }
    }

}

/* @brief 应用初始化：初始化 SDK 并启动位置控制示例线程
 * @return RT_EOK=成功，RT_ERROR=应答队列未就绪
 *
 * 通过 RT-Thread INIT_APP_EXPORT 在应用初始化阶段自动调用。
 * 注意：需要 Parse_Init() 先执行，应答队列 rs_res_mq 才可用。 */
int rs_ronin_app_init(void)
{
    rt_thread_t tid3 = RT_NULL;

    /* 初始化协议配置（控制标志、加密、CmdType） */
    dji_rs_sdk_init();

    /* 等待解析线程初始化完成 */
    rt_thread_mdelay(1000);
    /* 检查应答队列是否已创建 */
    if (rs_res_mq == RT_NULL)
    {
        rt_kprintf("rs_res_mq not ready, Parse_Init must run first!\n");
        return RT_ERROR;
    }

    /* 创建位置控制线程：栈 2048B、优先级 11、时间片 10 */
    tid3 = rt_thread_create("loc_ctrl", location_ctrl, RT_NULL, 2048, 11, 10);
    if(tid3 != RT_NULL)
        rt_thread_startup(tid3);

    return RT_EOK;
}INIT_APP_EXPORT(rs_ronin_app_init);




