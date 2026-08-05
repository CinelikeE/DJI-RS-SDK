/**
 * @file Handle.c
 * @brief CAN 总线收发模块实现
 *
 * 工作流程：
 *  1. rt_Can_init() 查找并打开 can0 设备，创建接收消息队列与信号量；
 *  2. CAN 接收中断触发 can_rx_call() 释放信号量，唤醒 can_rx_thread；
 *  3. can_rx_thread 读取一帧 CAN 数据并投入 can_rx_mq 消息队列；
 *  4. send_data() 将任意长度协议帧按 8 字节拆分，以 0x223 帧 ID 逐帧发送。
 *
 * 若定义 DJI_RS_SDK_CAN_LOOPBACK，则把 CAN 配置为回环模式（自测用）。
 */
#include "Handle.h"

/* 需要 CAN 回环自测时定义 DJI_RS_SDK_CAN_LOOPBACK */
#ifdef DJI_RS_SDK_CAN_LOOPBACK
#define Can_Mode_Loopback
#endif

#define CAN_DEV_NAME "can0"
#define CAN_MQ_MSG_NUM 20  /* 消息队列容量 */

static struct rt_semaphore rx_sem; /* 接收信号量：中断回调释放，接收线程等待 */
static rt_device_t can_dev;        /* CAN 设备句柄 */
rt_mq_t can_rx_mq;                 /* 接收消息队列句柄（全局，供解析线程使用） */

/* @brief CAN 接收中断回调
 *
 * 只释放信号量，不在此处读取数据（中断上下文要求处理尽量简短）；
 * 实际读帧操作由 can_rx_thread 在信号量就绪后完成。 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem); /* 通知接收线程有新数据到达 */
    return RT_EOK;
}

/* @brief CAN 接收线程
 *
 * 注册接收回调、配置硬件滤波（仅接收 0x222），
 * 然后循环等待信号量并读取 CAN 帧，放入 can_rx_mq 消息队列。 */
static void can_rx_thread(void *parameter)
{
    struct rt_can_msg rxmsg = {0};
    CanMsg mq_msg;

    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(can_dev, can_rx_call);

#ifdef RT_CAN_USING_HDR
    rt_err_t res;
    struct rt_can_filter_item items[1] =
    {
        RT_CAN_FILTER_STD_INIT(RS_CAN_RX_ID, RT_NULL, RT_NULL),  /* std, match ID:0x222 */
    };
    struct rt_can_filter_config cfg = {1, 1, items};
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);
#endif

    while (1)
    {
        /* hdr 值为 -1，表示直接从 uselist 链表读取数据 */
        rxmsg.hdr_index = -1;
        /* 阻塞等待接收信号量 */
        (void)rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        /* 从 CAN 读取一帧数据 */
        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));

        /* 只关心云台应答(0x222)与自环/推送(0x223) */
        if ((rxmsg.id == RS_CAN_RX_ID) || (rxmsg.id == RS_CAN_TX_ID))
        {
            /* 转存为 CanMsg 结构并投入消息队列，供解析线程使用 */
            mq_msg.id = rxmsg.id;
            mq_msg.len = rxmsg.len;
            rt_memcpy(mq_msg.data, rxmsg.data, rxmsg.len);
            rt_mq_send(can_rx_mq, &mq_msg, sizeof(CanMsg));
        }
    }
}

/* @brief 初始化 CAN 总线收发模块
 * @return RT_EOK=成功，RT_ERROR=失败
 *
 * 步骤：查找设备 -> 创建消息队列 -> 初始化信号量 -> 打开设备
 * （可选回环模式）-> 创建并启动接收线程。
 * 通过 RT-Thread INIT_DEVICE_EXPORT 在设备初始化阶段自动调用。 */
int rt_Can_init(void)
{
    rt_err_t res;
    rt_thread_t thread;
    char can_name[RT_NAME_MAX];

    rt_strncpy(can_name, CAN_DEV_NAME, RT_NAME_MAX); /* 拷贝设备名到本地缓冲 */

    /* 按名称查找 CAN 设备 */
    can_dev = rt_device_find(can_name);
    if (!can_dev)
    {
        rt_kprintf("find %s failed!\n", can_name);
        return RT_ERROR;
    }

    /* 创建 CAN 接收消息队列，容量 20 条 */
    can_rx_mq = rt_mq_create("can_rx_mq",
                             sizeof(CanMsg),
                             CAN_MQ_MSG_NUM,
                             RT_IPC_FLAG_PRIO);
    if (can_rx_mq == RT_NULL)
    {
        rt_kprintf("create can rx mq failed!\n");
        return RT_ERROR;
    }

    /* 初始化接收信号量（初始计数为 0） */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_PRIO);

    /* 以中断收发模式打开 CAN 设备 */
    res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    if (res != RT_EOK)
    {
        rt_kprintf("can dev open failed: %d\n", res);
        return RT_ERROR;
    }

#ifdef Can_Mode_Loopback
    /* 自测模式：配置为 CAN 回环 */
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, RT_CAN_MODE_LOOPBACK);
    if (res != RT_EOK)
        rt_kprintf("can set loopback mode failed: %d\n", res);
#endif

    /* 创建接收线程：栈 1024B、优先级 25、时间片 10 */
    thread = rt_thread_create("can_rx", can_rx_thread, RT_NULL, 1024, 25, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        rt_kprintf("create can_rx thread failed!\n");
    }
    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_Can_init);

/* @brief 发送任意长度的协议帧数据
 * @param data     待发送数据（完整协议帧）
 * @param data_len 数据总长度
 * @return true=全部发送成功，false=失败
 *
 * CAN 单帧最多承载 8 字节，因此把数据拆成多帧，
 * 每帧使用相同的标准帧 ID 0x223 发送。 */
bool send_data(uint8_t *data, uint16_t data_len)
{
    struct rt_can_msg msg = {0};
    rt_size_t size;
    uint16_t offset = 0;

    if (data_len == 0)
        return false;

    /* 循环拆分发送，直到全部数据发送完毕 */
    while (offset < data_len)
    {
        /* 本帧长度：剩余不足 8 字节时按剩余长度发送 */
        uint8_t frame_len = (data_len - offset >= 8) ? 8 : (uint8_t)(data_len - offset);

        msg.id  = RS_CAN_TX_ID; /* 标准帧，ID 0x223 */
        msg.ide = RT_CAN_STDID; /* 标准帧格式 */
        msg.rtr = RT_CAN_DTR;   /* 数据帧（非远程帧） */
        msg.len = frame_len;    /* 本帧有效字节数 */
        for (uint8_t j = 0; j < frame_len; ++j)
            msg.data[j] = data[offset + j];

        /* 写一帧到 CAN 设备；返回 0 表示写入失败 */
        size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
        if (size == 0)
        {
            rt_kprintf("can dev write data failed!\n");
            return false;
        }
        /* 偏移前进，继续发送剩余数据 */
        offset += frame_len;
    }

    return true;
}
