#include "CmdParse.h"

static FrameBuffer RScmd;
rt_mq_t rs_res_mq;
#define RS_RES_MQ_MSG_NUM 20  // 消息队列容量

static RS_Msg mq_msq;
void FrameParse(uint8_t *data){
    switch (data[12]) {
        case 0x0E:

            switch (data[13]) {
                case 0x00:
                    mq_msq.seq[0]  = data[8];
                    mq_msq.seq[1]  = data[9];

                    mq_msq.data[0] = data[14];

                    rt_mq_send(rs_res_mq, &mq_msq, sizeof(RS_Msg));
                    break;

                default:
                    break;
            }

            break;
        default:
            break;
    }



}


static void FrameCombine(uint8_t *data){
    uint8_t i = 0;
    crc16_t crc16;
    crc32_t crc32;
    switch(RScmd.state){
/*******************************************/
    case FRAME_IDLE:
      //  rt_kprintf("FRAME_IDLE\r\n");

        if ((data[0] == 0xAA) &&
            (data[4] == enc ) &&
            (data[5] == 0x00) &&
            (data[6] == 0x00) &&                  //第一帧判断,如果是第一帧，切换sof found
            (data[7] == 0x00) ){
            RScmd.state = FRAME_SOF_FOUND;
#ifdef debug
            rt_kprintf("FRAME_SOF_FOUND\r\n");
#endif
            RScmd.len_max = data[1];
            for (i = 0; i < 8; ++i) {
                RScmd.buffer[i] = data[i];
            }
            RScmd.len = 8;
            RScmd.state = FRAME_RECEIVING; //缓冲第一帧数据，全帧第二位是帧长，len计数位+8
        }

    break;
 /*******************************************/
    case FRAME_SOF_FOUND:
                                                //没有意义，空置减少编译器警告

    break;

 /*******************************************/
    case FRAME_RECEIVING:
#ifdef debug
        rt_kprintf("FRAME_RECEIVING\r\n");
#endif
        for (i = 0; i < 8; ++i) {
            RScmd.buffer[(RScmd.len + i)] = data[i];
            }
            RScmd.len +=8;

        if(RScmd.len == 16){
            crc16  = crc16_init();
            crc16  = crc16_update(crc16, RScmd.buffer, 10);
            crc16  = crc16_finalize(crc16);                                //crc16校验

            if ( (RScmd.buffer[10] != ( crc16       & 0xff)) ||
                 (RScmd.buffer[11] != ((crc16 >> 8) & 0xff)) ){
                rt_kprintf("receive crc16 failed\r\n");                   //如果校验结果对不上，弃掉数据重置缓冲区
                rt_memset(RScmd.buffer, 0, sizeof(RScmd.buffer));
                RScmd.len     =  0;
                RScmd.len_max =  0;
                RScmd.state   = FRAME_IDLE;
            }
#ifdef debug
            else {
                rt_kprintf("receive crc16 complete\r\n");
            }
#endif
        }

        if(RScmd.len >= RScmd.len_max)
        {

            crc32  = crc32_init();
            crc32  = crc32_update(crc32, RScmd.buffer,
                                         RScmd.len_max-4 );
            crc32  = crc32_finalize(crc32);                                                             //crc32校验



            if ( (RScmd.buffer[(RScmd.len_max - 4)] != ( crc32        & 0xff)) ||
                 (RScmd.buffer[(RScmd.len_max - 3)] != ((crc32 >> 8 ) & 0xff)) ||
                 (RScmd.buffer[(RScmd.len_max - 2)] != ((crc32 >> 16) & 0xff)) ||
                 (RScmd.buffer[(RScmd.len_max - 1)] != ((crc32 >> 24) & 0xff)) ){


                rt_kprintf("receive crc32 failed\r\n");                   //如果校验结果对不上，弃掉数据重置缓冲区


#ifdef debug
                rt_kprintf("%x  %x  %x  %x\r\n",
                        RScmd.buffer[RScmd.len_max-4],
                        RScmd.buffer[RScmd.len_max-3],
                        RScmd.buffer[RScmd.len_max-2],
                        RScmd.buffer[RScmd.len_max-1]);
                rt_kprintf("%x  %x  %x  %x\r\n",
                        ( crc32        & 0xff),
                        ((crc32 >> 8 ) & 0xff),
                        ((crc32 >> 16) & 0xff),
                        ((crc32 >> 24) & 0xff));

#endif
                rt_memset(RScmd.buffer, 0, sizeof(RScmd.buffer));
                RScmd.len     =  0;
                RScmd.len_max =  0;
                RScmd.state   = FRAME_IDLE;
            }
            else {

#ifdef debug
                rt_kprintf("receive crc32 complete\r\n");
#endif

            RScmd.state = FRAME_COMPLETE; //缓存余后数据，比较len和len max，若计数大于等于len max，单帧数接收完毕
            }
        }
        break;

/*******************************************/
     case FRAME_COMPLETE:
                                                        //没有意义，空置减少编译器警告

     break;


    }


}

void Parse(){
    CanMsg msg;
    rt_err_t res;

    // 无限循环接收消息队列数据
    while (1)
    {
        if(RScmd.state == FRAME_COMPLETE){

#ifdef debug
            rt_kprintf("can ID : %x-- ",msg.id);
            for (int i = 0; i < RScmd.len_max; ++i) {
                rt_kprintf("%x   ", RScmd.buffer[i] );
            }
            rt_kprintf("\r\n");


            rt_kprintf("FRAME_COMPLETE\r\n");
#endif
            FrameParse(RScmd.buffer);                         //接收完成后送入parse函数应答帧解析
            rt_memset(RScmd.buffer, 0, sizeof(RScmd.buffer)); //清空缓冲区数组
            RScmd.len     =  0;
            RScmd.len_max =  0;
            RScmd.state   = FRAME_IDLE;                       //切换空状态



        }


        // 阻塞等待消息队列数据
        res = rt_mq_recv(can_rx_mq, &msg, sizeof(CanMsg), 2000);  //计时200tick阻塞
        if ((res != RT_ERROR)||(res != RT_ETIMEOUT))
        {
            FrameCombine(msg.data);
        }
        else if (res == RT_ETIMEOUT) {                          //如果超时未能接收数据，清空缓冲区，重置计数位
            rt_memset(RScmd.buffer, 0, sizeof(RScmd.buffer));
            RScmd.len     =  0;
            RScmd.len_max =  0;
            RScmd.state   = FRAME_IDLE;
        }
        else if(res == RT_ERROR) {
            rt_kprintf("Receive CAN msg failed: %d\n", res);
        }


    }
}


//#ifdef NoResponse
int Parse_Init()
{
    rt_thread_t tid = rt_thread_create("cmd_parse", Parse, RT_NULL, 1024, 24, 10);
    if (tid != RT_NULL)
    {

        // 初始化消息队列
        rs_res_mq = rt_mq_create("rs_res_mq",
                                 sizeof(RS_Msg),
                                 RS_RES_MQ_MSG_NUM,
                                 RT_IPC_FLAG_PRIO);
        if (can_rx_mq == RT_NULL)
        {
            rt_kprintf("create rs_res_mq failed!\n");
            return RT_ERROR;
        }



        rt_thread_startup(tid);
        //rt_kprintf("Parse thread created successfully!\n");
    }
    else
    {
        rt_kprintf("Create parse thread failed!\n");
    }
    return RT_EOK;
}INIT_APP_EXPORT(Parse_Init);

//#endif
