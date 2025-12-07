#include "Handle.h"

#define Can_Mode_Loopback

#define CAN_DEV_NAME "can0" /* CAN 设备名称 */

static struct rt_semaphore rx_sem; /* 用于接收消息的信号量 */
static rt_device_t can_dev;        /* CAN 设备句柄 */

/* 接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}

static void can_rx_thread(void *parameter)
{
    int i;
    rt_err_t res;
    struct rt_can_msg rxmsg = {0};

    uint8_t data_lenth = 0;
    uint8_t total_frames = 0;
    uint8_t lenth = 0;
    uint8_t frames = 0;
    uint8_t left_lenth = 0;

    uint8_t rx_buf[50] = {0};   //can数据缓冲区



    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(can_dev, can_rx_call);
    //rt_kprintf("can_rx_thread\n");
#ifdef RT_CAN_USING_HDR

    struct rt_can_filter_item items[1] =
    {
        //RT_CAN_FILTER_ITEM_INIT(0x01, 0, 0, 1, 0x01, RT_NULL, RT_NULL),
        RT_CAN_FILTER_STD_INIT(0x233,RT_NULL,RT_NULL),
        /* 过滤 ID 为 0x01,match ID:0x100~0x1ff，hdr 为 - 1，设置默认过滤表 */
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
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        /* 从 CAN 读取一帧数据 */
        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));


        if( (rxmsg.data[0] == 0xAA) && (data_lenth == 0) ){         //抓取帧头并确保是帧头

            data_lenth = rxmsg.data[1];                             //总帧长
            total_frames = rxmsg.data[1] / 8;                       //总帧数
            left_lenth = data_lenth % 8;                            //非完整数据帧（最后一帧）

            if(left_lenth != 0)
                total_frames ++;
        }

        if ( (frames == (total_frames - 1)) && (left_lenth != 0) ) {//抓取 最后一帧&&非完整数据帧 到缓冲区
            for (i = 0; i < left_lenth; ++i) {
                rx_buf[lenth] = rxmsg.data[i];
                rt_kprintf("%2x", rx_buf[lenth]);
                lenth++;
            }
            frames++;
        }

        if(frames != total_frames){
            for (i = 0; i < 8; ++i) {
                rx_buf[lenth] = rxmsg.data[i];                      //抓取完整can数据帧到缓冲区
                rt_kprintf("%2x", rx_buf[lenth]);
                lenth++;                                            //长度计数位
            }

            rt_kprintf("\r\n");
            frames++;                                               //帧数计数位
        }
        else {


            data_lenth = 0;                                         //帧数、帧长数据和计数位清零，导出帧数据
            total_frames = 0;
            lenth = 0;
            frames = 0;
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

    /* 初始化 CAN 接收信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);

    /* 以中断接收及发送方式打开 CAN 设备 */
    res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, RT_CAN_MODE_LOOPBACK);
    RT_ASSERT(res == RT_EOK);

#ifdef Can_Mode_Loopback
    rt_device_control(can_dev,RT_CAN_CMD_SET_MODE,RT_CAN_MODE_LOOPBACK);    //设置回环模式
#endif

    /* 创建数据接收线程 */
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

    uint32_t frame_num;                                     //总帧数
    uint32_t full_frame_num = data_len / 8;                 //满数据帧数
    uint32_t left_frame_len = data_len % 8;                 //剩余数据长度


    if (left_frame_len == 0)
        frame_num = full_frame_num;
    else
        frame_num = full_frame_num + 1;                     //frame number judge 总帧数判断

    uint32_t data_offset   = 0;                             //数据计数位
    uint32_t frame_len        ;                             //单帧长度
    uint32_t left_data_len = data_len;                      //剩余数据长度

    for (int i = 0; i < frame_num; i++) {

        if (left_data_len >= 8)
            frame_len = 8;
        else
            frame_len = left_data_len;                      //single frame length judge 单帧长度判断

        uint8_t *buf  = (uint8_t*)rt_malloc(frame_len);//single frame send data 单帧发送数据

        for (int j = 0; j < frame_len; j++) {

            buf[j] = data[data_offset + j];

        }
        uint8_t *send_buf = buf;

/** here is how to send data frame with can in stm32 with different method **********************************************/

        msg.id  = 0x222;                                        // ID 为 0x222
        msg.ide = RT_CAN_STDID;                                 // 标准格式
        msg.rtr = RT_CAN_DTR;                                   // 数据帧

        msg.len = frame_len;                                // 数据长度为 实际帧数据长度

        for (int j = 0; j < frame_len; ++j) {
            msg.data[j] = send_buf[j];
        }

       // size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
        size = rt_device_write(can_dev, send_buf, &msg, sizeof(msg));

        if (size == 0)
        {
            rt_kprintf("can dev write data failed!\n");
            return false;
        }


/************************************************************************************************************************/
        rt_free(send_buf);
        left_data_len -= 8;                                 //left data length = last left data length - 8  剩余数据 = 当前剩余数据 - 8
        data_offset += 8;                                   //
        rt_thread_mdelay(2);
    }

    return true;

}





