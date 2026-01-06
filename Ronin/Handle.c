#include "Handle.h"


#ifdef debug
#define Can_Mode_Loopback
#endif

#define CAN_DEV_NAME "can0"
#define CAN_MQ_MSG_NUM 20  // 消息队列容量

static struct rt_semaphore rx_sem;
static rt_device_t can_dev;
rt_mq_t can_rx_mq;  // 消息队列句柄

static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{

    rt_sem_release(&rx_sem);

    return RT_EOK;
}

static void can_rx_thread(void *parameter)
{
    int i;
    rt_err_t res;
    struct rt_can_msg rxmsg = {0};
    CanMsg mq_msg;  // 用于消息队列的消息

    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(can_dev, can_rx_call);

#ifdef RT_CAN_USING_HDR
    struct rt_can_filter_item items[1] =
    {

        RT_CAN_FILTER_STD_INIT(0x222, RT_NULL, RT_NULL),                  /* std,match ID:0x222，hdr 为 - 1 */

    };
    struct rt_can_filter_config cfg = {1, 1, items}; /* 一共有 1 个过滤表 */
    /* 设置硬件过滤表 */
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);
#endif

    while (1)
    {
        /* hdr 值为 - 1，表示直接从 uselist 链表读取数据 */
        rxmsg.hdr_index = -1;
        /* 阻塞等待接收信号量 */
        res = rt_sem_take(&rx_sem, RT_WAITING_FOREVER);

        /* 从 CAN 读取一帧数据 */
        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));



        if( (rxmsg.id == 0x222)||(rxmsg.id == 0x223) ){

        // 填充消息队列数据
        mq_msg.id = rxmsg.id;
        mq_msg.len = rxmsg.len;
        rt_memcpy(mq_msg.data, rxmsg.data, rxmsg.len);
        // 发送到消息队列，非阻塞方式
        rt_mq_send(can_rx_mq, &mq_msg, sizeof(CanMsg));



        }
    }
}


int rt_Can_init(){

    rt_err_t res;
    rt_size_t  size;
    rt_thread_t thread;
    char can_name[RT_NAME_MAX];

    rt_strncpy(can_name, CAN_DEV_NAME, RT_NAME_MAX);

    can_dev = rt_device_find(can_name);
    if (!can_dev)
    {
        rt_kprintf("find %s failed!\n", can_name);
        return RT_ERROR;
    }

    // 初始化消息队列
    can_rx_mq = rt_mq_create("can_rx_mq",
                             sizeof(CanMsg),
                             CAN_MQ_MSG_NUM,
                             RT_IPC_FLAG_PRIO);
    if (can_rx_mq == RT_NULL)
    {
        rt_kprintf("create can rx mq failed!\n");
        return RT_ERROR;
    }


    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_PRIO);


    res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);

    RT_ASSERT(res == RT_EOK);

#ifdef Can_Mode_Loopback
    rt_device_control(can_dev,RT_CAN_CMD_SET_MODE,RT_CAN_MODE_LOOPBACK);
#endif


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
}INIT_DEVICE_EXPORT(rt_Can_init);

uint32_t send_data(uint8_t *data, uint8_t data_len){

    struct rt_can_msg msg = {0};
    rt_size_t size;

    uint32_t frame_num;
    uint32_t full_frame_num = data_len / 8;                 //计算完整帧数
    uint32_t left_frame_len = data_len % 8;                 //非完整帧字节长度


    if (left_frame_len == 0)
        frame_num = full_frame_num;
    else
        frame_num = full_frame_num + 1;                     //frame number judge

    uint32_t data_offset   = 0;
    uint32_t frame_len        ;
    uint32_t left_data_len = data_len;

    for (int i = 0; i < frame_num; i++) {

        if (left_data_len >= 8)
            frame_len = 8;
        else
            frame_len = left_data_len;                      //single frame length judge

        uint8_t *buf  = (uint8_t*)rt_malloc(frame_len);     //single frame send data

        for (int j = 0; j < frame_len; j++) {

            buf[j] = data[data_offset + j];

        }
        uint8_t *send_buf = buf;

/**********************can数据发送**************************/

        msg.id  = 0x223;
        msg.ide = RT_CAN_STDID;
        msg.rtr = RT_CAN_DTR;

        msg.len = frame_len;

        for (int j = 0; j < frame_len; ++j) {
            msg.data[j] = send_buf[j];
        }

       // size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
        size = rt_device_write(can_dev, 0, &msg, sizeof(msg));

        if (size == 0)
        {
            rt_kprintf("can dev write data failed!\n");
            return false;
        }


/************************************************************************************************************************/
        rt_free(send_buf);
        left_data_len -= 8;                                 //left data length = last left data length - 8
        data_offset += 8;

    }

    return true;

}





