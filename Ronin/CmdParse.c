#include "CmdParse.h"
#define debug
Buffer FrameBuffer;

void FrameParse(uint8_t *data){

}


static void FrameCombine(uint8_t *data){
    uint8_t i = 0;
    switch(FrameBuffer.state){
/*******************************************/
    case(FRAME_IDLE):
      //  rt_kprintf("FRAME_IDLE\r\n");

        if ((data[0] == 0xAA) &&
            (data[4] == ENC ) &&
            (data[5] == 0x00) &&
            (data[6] == 0x00) &&                  //第一帧判断
            (data[7] == 0x00) ){
            FrameBuffer.state = FRAME_SOF_FOUND;
#ifdef debug
            rt_kprintf("FRAME_SOF_FOUND\r\n");
#endif
            FrameBuffer.len_max = data[1];
            for (i = 0; i < 8; ++i) {
                FrameBuffer.buffer[i] = data[i];
            }
            FrameBuffer.len = 8;
            FrameBuffer.state = FRAME_RECEIVING;
        }

    break;
 /*******************************************/
    case(FRAME_SOF_FOUND):


    break;

 /*******************************************/
    case(FRAME_RECEIVING):
#ifdef debug
        rt_kprintf("FRAME_RECEIVING\r\n");
#endif
        for (i = 0; i < 8; ++i) {
            FrameBuffer.buffer[(FrameBuffer.len + i)] = data[i];
            }
            FrameBuffer.len +=8;
        if(FrameBuffer.len >= FrameBuffer.len_max)
        {
            FrameBuffer.state = FRAME_COMPLETE;

        }
        break;

/*******************************************/
     case(FRAME_COMPLETE):


     break;


    }


}

void Parse(){
    CanMsg msg;
    rt_err_t res;

    // 无限循环接收消息队列数据
    while (1)
    {
        if(FrameBuffer.state == FRAME_COMPLETE){

#ifdef debug
            rt_kprintf("can ID : %x ",msg.id);
            for (int i = 0; i < FrameBuffer.len_max; ++i) {
                rt_kprintf("%x   ", FrameBuffer.buffer[i] );
            }
            rt_kprintf("\r\n");


            rt_kprintf("FRAME_COMPLETE\r\n");
#endif
            FrameParse(FrameBuffer.buffer);
            rt_memset(FrameBuffer.buffer, 0, sizeof(FrameBuffer.buffer));
            FrameBuffer.len     =  0;
            FrameBuffer.len_max =  0;
            FrameBuffer.state   = FRAME_IDLE;



        }
        // 阻塞等待消息队列数据
        res = rt_mq_recv(can_rx_mq, &msg, sizeof(CanMsg), 200);
        if ((res != RT_ERROR)||(res != RT_ETIMEOUT))
        {
            FrameCombine(msg.data);
        }
        else if (res == RT_ETIMEOUT) {
            rt_memset(FrameBuffer.buffer, 0, sizeof(FrameBuffer.buffer));
            FrameBuffer.len     =  0;
            FrameBuffer.len_max =  0;
            FrameBuffer.state   = FRAME_IDLE;
        }
        else if(res == RT_ERROR) {
            rt_kprintf("Receive CAN msg failed: %d\n", res);
        }


    }
}

int Parse_Init()
{
    rt_thread_t tid = rt_thread_create("cmd_parse", Parse, RT_NULL, 1024, 24, 10);
    if (tid != RT_NULL)
    {


        rt_thread_startup(tid);
        rt_kprintf("Parse thread created successfully!\n");
    }
    else
    {
        rt_kprintf("Create parse thread failed!\n");
    }
    return RT_EOK;
}INIT_APP_EXPORT(Parse_Init);

