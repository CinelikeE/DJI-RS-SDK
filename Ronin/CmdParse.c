#include "CmdParse.h"


// 初始化帧缓冲区
static FrameBuffer frame_buf = {
    .len = 0,
    .state = FRAME_IDLE
};

// 检查帧完整性（SOF + 长度 + 数据 + CRC）
static bool is_frame_complete(FrameBuffer *buf)
{
    if (buf->len < 3) return false;  // 至少需要SOF(1) + 长度(2)

    // 从帧头获取总长度（根据协议，这里是Ver/Length字段的前10bit）
    uint16_t frame_total_len = (buf->buffer[1] << 8) | buf->buffer[2];
    frame_total_len &= 0x03FF;  // 只取低10位

    return (buf->len >= frame_total_len);
}

// 验证CRC
static bool verify_crc(FrameBuffer *buf)
{
    if (buf->len < 7) return false;  // 至少需要到CRC-16之前的字段

    uint16_t crc16_calc, crc16_recv;
    uint32_t crc32_calc, crc32_recv;
    uint16_t frame_total_len = (buf->buffer[1] << 8) | buf->buffer[2] & 0x03FF;

    // 计算CRC16（SOF到SEQ字段）
    crc16_calc = crc16_init();
    crc16_calc = crc16_update(crc16_calc, buf->buffer, 10);  // SOF(1)+Ver/Len(2)+...+SEQ(2)
    crc16_calc = crc16_finalize(crc16_calc);
    crc16_recv = (buf->buffer[11] << 8) | buf->buffer[10];  // CRC-16在第10-11字节
    if (crc16_calc != crc16_recv) return false;

    // 计算CRC32（整个帧除CRC32外）
    crc32_calc = crc32_init();
    crc32_calc = crc32_update(crc32_calc, buf->buffer, frame_total_len - 4);
    crc32_calc = crc32_finalize(crc32_calc);
    crc32_recv = (buf->buffer[frame_total_len-1] << 24) |
                 (buf->buffer[frame_total_len-2] << 16) |
                 (buf->buffer[frame_total_len-3] << 8) |
                 buf->buffer[frame_total_len-4];
    return (crc32_calc == crc32_recv);
}

// 解析完整帧数据
static void parse_frame(FrameBuffer *buf)
{
    uint8_t cmd_type = buf->buffer[3];
    uint8_t cmd_set = buf->buffer[12];
    uint8_t cmd_id = buf->buffer[13];
    uint8_t *data = &buf->buffer[14];
    uint16_t data_len = (buf->buffer[1] << 8 | buf->buffer[2] & 0x03FF) - 18;  // 总长度 - 固定头部长度

    rt_kprintf("Parse frame: CmdType=0x%x, CmdSet=0x%x, CmdID=0x%x, DataLen=%d\n",
               cmd_type, cmd_set, cmd_id, data_len);

    // 根据不同的CmdSet和CmdID进行具体解析
    switch (cmd_set)
    {
        case 0x0E:  // 云台控制相关命令
            switch (cmd_id)
            {
                case 0x00:  // 位置控制反馈
                    // 解析位置反馈数据
                    break;
                case 0x01:  // 速度控制反馈
                    // 解析速度反馈数据
                    break;
                default:
                    rt_kprintf("Unknown CmdID: 0x%x\n", cmd_id);
                    break;
            }
            break;
        // 其他CmdSet的处理...
        default:
            rt_kprintf("Unknown CmdSet: 0x%x\n", cmd_set);
            break;
    }
}

// 合并CAN帧数据
static void merge_can_frames(CanMsg *msg)
{
    // 只处理目标ID的帧（根据实际需求修改）
    if (msg->id != 0x222) return;

    switch (frame_buf.state)
    {
        case FRAME_IDLE:
            // 寻找帧起始符SOF（0xAA）
            if (msg->data[0] == 0xAA)
            {
                frame_buf.buffer[0] = 0xAA;
                frame_buf.len = 1;
                frame_buf.state = FRAME_SOF_FOUND;
            }
            break;

        case FRAME_SOF_FOUND:
        case FRAME_RECEIVING:
            // 累加数据到缓冲区
            if (frame_buf.len + msg->len <= sizeof(frame_buf.buffer))
            {
                rt_memcpy(&frame_buf.buffer[frame_buf.len], msg->data, msg->len);
                frame_buf.len += msg->len;
                frame_buf.state = FRAME_RECEIVING;

                // 检查帧是否完整
                if (is_frame_complete(&frame_buf))
                {
                    frame_buf.state = FRAME_COMPLETE;
                }
            }
            else
            {
                // 缓冲区溢出，重置
                rt_kprintf("Frame buffer overflow!\n");
                frame_buf.len = 0;
                frame_buf.state = FRAME_IDLE;
            }
            break;

        default:
            break;
    }

    // 处理完整帧
    if (frame_buf.state == FRAME_COMPLETE)
    {
        if (verify_crc(&frame_buf))
        {
            parse_frame(&frame_buf);
        }
        else
        {
            rt_kprintf("Frame CRC error!\n");
        }
        // 重置缓冲区，准备接收下一帧
        frame_buf.len = 0;
        frame_buf.state = FRAME_IDLE;
    }
}


void Parse(){
    CanMsg msg;
    rt_err_t res;

    // 无限循环接收消息队列数据
    while (1)
    {
        // 阻塞等待消息队列数据
        res = rt_mq_recv(can_rx_mq, &msg, sizeof(CanMsg), RT_WAITING_FOREVER);
        if ((res != RT_ERROR)||(res != RT_ETIMEOUT))
        {
            //merge_can_frames(&msg);  // 合并CAN帧
            rt_kprintf("can ID : %x ",msg.id);
            for (int i = 0; i < 8; ++i) {
                rt_kprintf("%x   ", msg.data[i] );
            }
            rt_kprintf("\r\n");

        }
        else
        {
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

