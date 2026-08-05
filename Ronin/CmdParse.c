/**
 * @file CmdParse.c
 * @brief DJI RS 协议帧解析实现
 *
 * 工作流程：
 *  1. Parse() 线程从 can_rx_mq 队列取出 CAN 消息；
 *  2. FrameCombine() 按协议状态机把多条 8 字节 CAN 帧拼成完整协议帧，
 *     并依次校验 SOF、ENC、CRC16 与 CRC32；
 *  3. FrameParse() 把完整帧解析为 RS_Msg：
 *       - 应答帧送入 rs_res_mq，供等待应答的线程匹配；
 *       - 参数推送/校准状态推送帧直接调用已注册的回调处理。
 */
#include "CmdParse.h"

static FrameBuffer RScmd;
rt_mq_t rs_res_mq;
#define RS_RES_MQ_MSG_NUM 20
/* 上面四行为解析模块的全局资源：
 *  RScmd   - 协议帧接收缓冲（含组帧状态机）
 *  rs_res_mq - 应答消息队列（解析线程 -> 等待应答的线程）
 *  mq_msq  - 解析结果复用缓冲区（单消费者）
 *  push_cb - 参数推送回调，由用户注册 */

static RS_Msg mq_msq;
static rs_push_callback_t push_cb = RT_NULL;

/* 完整帧解析：data 指向 RScmd.buffer（一帧完整数据） */
/* @brief 将一条完整协议帧解析为 RS_Msg 并分发
 * @param data 指向完整协议帧首地址（RScmd.buffer）
 * 应答帧会进入 rs_res_mq；参数推送/校准状态推送帧直接回调，
 * 不再进入应答队列，避免被等待应答的线程误消费。 */
void FrameParse(uint8_t *data)
{
    /* 帧长度位于第 1、2 字节（小端），即 Ver/Length 字段低 10 位 */
    uint16_t frame_len = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    /* 数据段长度 = 整帧长度 - 16：即 CmdSet(1)+CmdID(1)+CmdData(n) */
    uint16_t data_seg_len = (frame_len > 16) ? (frame_len - 16) : 0; /* CmdSet+CmdID+CmdData */
    /* CmdData 长度 = 数据段长度 - 2 */
    uint16_t payload_len = (data_seg_len > 2) ? (data_seg_len - 2) : 0;
    rt_err_t err;

    /* 从固定偏移位置提取帧字段 */
    mq_msq.seq[0] = data[8];
    mq_msq.seq[1] = data[9];
    mq_msq.frame_type = (data[3] >> 5) & 0x01;
    mq_msq.cmd_set = data[12];
    mq_msq.cmd_id  = data[13];
    mq_msq.len = (payload_len > sizeof(mq_msq.data)) ? sizeof(mq_msq.data) : payload_len;
    /* 拷贝 CmdData 部分（应答帧首字节为返回码） */
    rt_memcpy(mq_msq.data, &data[14], mq_msq.len);

    /* 推送帧（0x0E/0x08 参数推送、0x0E/0x10 校准状态推送）由回调处理，不走应答队列 */
    if (!(mq_msq.cmd_set == 0x0E &&
          (mq_msq.cmd_id == RS_CMD_PUSH_PARAM || mq_msq.cmd_id == RS_CMD_PUSH_CALIB)))
    {
        /* 普通应答帧发送到应答队列 */
        err = rt_mq_send(rs_res_mq, &mq_msq, sizeof(RS_Msg));
        if (err != RT_EOK)
            rt_kprintf("rs_res_mq send failed: %d\n", err);
    }

    /* 参数推送 / 校准状态推送 */
    if (push_cb != RT_NULL &&
        mq_msq.cmd_set == 0x0E &&
        (mq_msq.cmd_id == RS_CMD_PUSH_PARAM || mq_msq.cmd_id == RS_CMD_PUSH_CALIB))
    {
        /* 调用用户注册的回调；msg 指向内部静态缓冲，回调内如需保留数据请立即拷贝 */
        push_cb(&mq_msq);
    }
}

/* @brief 注册参数推送/校准状态推送回调
 * @param cb 回调函数，参数为解析后的 RS_Msg */
void rs_set_push_callback(rs_push_callback_t cb)
{
    /* 直接保存回调指针，后续收到推送帧时调用 */
    push_cb = cb;
}

/* @brief 将 2.3.4.9 参数推送消息解析为 RS_PushParam 结构体
 * @param msg 推送消息（len 至少 22 字节）
 * @param out 输出结构体，解析前会被清零 */
void rs_parse_push_param(const RS_Msg *msg, RS_PushParam *out)
{
    const uint8_t *d;

    /* 参数合法性检查 */
    if (msg == RT_NULL || out == RT_NULL)
        return;

    /* 输出结构清零，避免残留旧数据 */
    rt_memset(out, 0, sizeof(RS_PushParam));
    /* 推送数据不足 22 字节时无法解析，直接返回 */
    if (msg->len < 22)
        return;

    d = msg->data;
    /* 以下按协议 2.3.4.9 的固定字节序解析（角度单位为 0.1°） */
    out->valid_flag = d[0];
    out->yaw_angle   = (int16_t)((uint16_t)d[1]  | ((uint16_t)d[2]  << 8)); /* Yaw 角度（0.1°） */
    out->roll_angle  = (int16_t)((uint16_t)d[3]  | ((uint16_t)d[4]  << 8)); /* Roll 角度（0.1°） */
    out->pitch_angle = (int16_t)((uint16_t)d[5]  | ((uint16_t)d[6]  << 8)); /* Pitch 角度（0.1°） */
    out->yaw_joint_angle   = (int16_t)((uint16_t)d[7]  | ((uint16_t)d[8]  << 8)); /* Yaw 关节角（0.1°） */
    out->roll_joint_angle  = (int16_t)((uint16_t)d[9]  | ((uint16_t)d[10] << 8)); /* Roll 关节角（0.1°） */
    out->pitch_joint_angle = (int16_t)((uint16_t)d[11] | ((uint16_t)d[12] << 8)); /* Pitch 关节角（0.1°） */
    out->pitch_max = d[13]; /* Pitch 限位最大值 */
    out->pitch_min = d[14]; /* Pitch 限位最小值 */
    out->yaw_max   = d[15]; /* Yaw 限位最大值 */
    out->yaw_min   = d[16]; /* Yaw 限位最小值 */
    out->roll_max  = d[17]; /* Roll 限位最大值 */
    out->roll_min  = d[18]; /* Roll 限位最小值 */
    out->pitch_stiffness = d[19]; /* Pitch 电机力度（0~100） */
    out->yaw_stiffness   = d[20]; /* Yaw 电机力度（0~100） */
    out->roll_stiffness  = d[21]; /* Roll 电机力度（0~100） */
}

/* 将多帧 CAN 数据拼接为完整协议帧，并做 CRC16/CRC32 校验 */
/* @brief CAN 数据组帧状态机
 * @param data 一帧 8 字节 CAN 数据
 *
 * 状态机：
 *  FRAME_IDLE      -> 检查帧头（SOF/ENC/RES）与帧长，合法则开始收帧
 *  FRAME_RECEIVING -> 累积数据，16 字节时校验 CRC16，收满时校验 CRC32
 *  FRAME_COMPLETE  -> 完整帧已就绪，等待 Parse() 取走 */
static void FrameCombine(uint8_t *data)
{
    uint8_t i;
    uint16_t frame_len;
    crc16_t crc16;
    crc32_t crc32;

    switch (RScmd.state)
    {
    case FRAME_IDLE:
        /* 校验帧头：SOF=0xAA、ENC 与本地配置一致、RES 保留位全 0 */
        if ((data[0] == 0xAA) &&
            (data[4] == enc) &&
            (data[5] == 0x00) &&
            (data[6] == 0x00) &&
            (data[7] == 0x00))
        {
            /* 解析帧长字段（Ver/Length 低 10 位，小端） */
            frame_len = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
            /* 协议最小帧长 18 字节，且不能超过接收缓冲上限 */
            if (frame_len >= 18 && frame_len <= RS_MAX_FRAME_LEN)
            {
                RScmd.len_max = frame_len;
                /* 先把帧头 8 字节存入缓冲 */
                for (i = 0; i < 8; ++i)
                    RScmd.buffer[i] = data[i];
                RScmd.len = 8;
                RScmd.state = FRAME_RECEIVING; /* 帧头合法，进入收帧状态 */
            }
            else
            {
                /* 帧长非法，丢弃 */
                rt_kprintf("receive invalid frame len: %d\n", frame_len);
            }
        }
        break;

    case FRAME_RECEIVING:
        /* 把当前 8 字节 CAN 数据追加到帧缓冲（防越界） */
        for (i = 0; i < 8; ++i)
        {
            if ((RScmd.len + i) < sizeof(RScmd.buffer))
                RScmd.buffer[RScmd.len + i] = data[i];
        }
        RScmd.len += 8;

        /* 前 16 字节（SOF..CRC16 后的 4 字节数据）到达后校验 CRC16 */
        if (RScmd.len == 16)
        {
            /* 对前 10 字节（SOF..SEQ）计算 CRC16 */
            crc16 = crc16_init();
            crc16 = crc16_update(crc16, RScmd.buffer, 10);
            crc16 = crc16_finalize(crc16);
            /* 与帧中 CRC16 字段（第 10/11 字节，小端）比较 */
            if ((RScmd.buffer[10] != (crc16 & 0xFF)) ||
                (RScmd.buffer[11] != ((crc16 >> 8) & 0xFF)))
            {
                rt_kprintf("receive crc16 failed\r\n");
                /* 校验失败：清空缓冲并回到空闲态 */
                rt_memset(RScmd.buffer, 0, sizeof(RScmd.buffer));
                RScmd.len = 0;
                RScmd.len_max = 0;
                RScmd.state = FRAME_IDLE;
            }
        }

        /* 整帧到达后校验 CRC32 */
        if (RScmd.len >= RScmd.len_max && RScmd.len_max > 0)
        {
            /* 对整帧除尾部 CRC32 外的数据计算 CRC32 */
            crc32 = crc32_init();
            crc32 = crc32_update(crc32, RScmd.buffer, RScmd.len_max - 4);
            crc32 = crc32_finalize(crc32);
            /* 与帧尾 CRC32 字段（最后 4 字节，小端）比较 */
            if ((RScmd.buffer[RScmd.len_max - 4] != (crc32 & 0xFF)) ||
                (RScmd.buffer[RScmd.len_max - 3] != ((crc32 >> 8) & 0xFF)) ||
                (RScmd.buffer[RScmd.len_max - 2] != ((crc32 >> 16) & 0xFF)) ||
                (RScmd.buffer[RScmd.len_max - 1] != ((crc32 >> 24) & 0xFF)))
            {
                rt_kprintf("receive crc32 failed\r\n");
                /* 校验失败：清空缓冲并回到空闲态 */
                rt_memset(RScmd.buffer, 0, sizeof(RScmd.buffer));
                RScmd.len = 0;
                RScmd.len_max = 0;
                RScmd.state = FRAME_IDLE;
            }
            else
            {
                RScmd.state = FRAME_COMPLETE; /* 校验通过，整帧可用 */
            }
        }
        break;

    case FRAME_COMPLETE:
        break;

    default:
        break;
    }
}

/* @brief 命令解析线程入口
 * @param parameter 线程参数（未使用）
 *
 * 循环执行：
 *  1. 若已有完整帧则调用 FrameParse() 分发并复位缓冲；
 *  2. 从 can_rx_mq 阻塞接收 CAN 消息（2 秒超时）；
 *  3. 将每帧 8 字节数据送入 FrameCombine() 组帧。 */
void Parse(void *parameter)
{
    CanMsg msg;
    rt_err_t res;

    while (1)
    {
        /* 优先处理已拼好的完整协议帧 */
        if (RScmd.state == FRAME_COMPLETE)
        {
            FrameParse(RScmd.buffer);
            /* 处理完毕，清空缓冲并恢复空闲态 */
            rt_memset(RScmd.buffer, 0, sizeof(RScmd.buffer));
            RScmd.len = 0;
            RScmd.len_max = 0;
            RScmd.state = FRAME_IDLE;
        }

        /* 阻塞等待 CAN 接收消息队列（最多 2000ms） */
        res = rt_mq_recv(can_rx_mq, &msg, sizeof(CanMsg), 2000);
        /* rt_mq_recv 成功时返回实际接收字节数，失败返回负错误码 */
        if (res >= 0)
        {
            /* 收到一帧 8 字节 CAN 数据，送入组帧状态机 */
            FrameCombine(msg.data);
        }
        else if (res == -RT_ETIMEOUT)
        {
            /* 超时未收到新 CAN 数据，清空半包缓冲，防止残留数据干扰下一帧 */
            rt_memset(RScmd.buffer, 0, sizeof(RScmd.buffer));
            RScmd.len = 0;
            RScmd.len_max = 0;
            RScmd.state = FRAME_IDLE;
        }
        else
        {
            rt_kprintf("Receive CAN msg failed: %d\n", res);
        }
    }
}

/* @brief 初始化解析模块
 * @return RT_EOK=成功，RT_ERROR=应答队列创建失败
 *
 * 创建应答消息队列 rs_res_mq，并启动 cmd_parse 解析线程。
 * 通过 RT-Thread INIT_APP_EXPORT 在应用初始化阶段自动调用。 */
int Parse_Init(void)
{
    rt_thread_t tid;

    /* 初始化应答消息队列 */
    rs_res_mq = rt_mq_create("rs_res_mq",
                             sizeof(RS_Msg),
                             RS_RES_MQ_MSG_NUM,
                             RT_IPC_FLAG_PRIO);
    if (rs_res_mq == RT_NULL)
    {
        rt_kprintf("create rs_res_mq failed!\n");
        return RT_ERROR;
    }

    tid = rt_thread_create("cmd_parse", Parse, RT_NULL, 1024, 24, 10);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        rt_kprintf("Create parse thread failed!\n");
    }
    return RT_EOK;
}
INIT_APP_EXPORT(Parse_Init);
