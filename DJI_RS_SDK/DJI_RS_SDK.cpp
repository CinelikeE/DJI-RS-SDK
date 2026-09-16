/**
 * @file DJI_RS_SDK.cpp
 * @brief DJI RS 手持云台 SDK 对外接口实现（对应原 C 版 DJI_RS_SDK.c + DJI_RS_Set.c）
 *
 * 每个业务函数对应协议文档 2.3.x 的一条命令：
 *  - 需要应答的命令：sendAndWait() 发送并等待应答，返回云台返回码（0x00 成功）
 *    或本地错误码 RS_TIMEOUT(0x03) / RS_ERROR(0x04)；
 *  - 不需要应答的命令（摇杆/拨轮、自动校准、智能跟随等）：sendCmd() 直接发送。
 */
#include "DJI_RS_SDK.h"

/* 等待云台应答的默认超时时间（ms） */
#define RS_DEFAULT_TIMEOUT_MS 2000

/* ---------- 小端读写小工具 ---------- */

/**
 * @brief 把 16 位数值按小端（低字节在前）写入缓冲区
 * @param p 目标缓冲区，至少 2 字节
 * @param v 要写入的 16 位数值
 * @note 协议里所有多字节字段都是小端，例如 900(0x0384) 会被写成 84 03。
 */
static inline void putU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

/**
 * @brief 从缓冲区按小端读出 16 位数值
 * @param p 源缓冲区，至少 2 字节
 * @return 拼装出的 16 位数值（不做符号扩展；需要负数时由调用方转成 int16_t）
 */
static inline uint16_t getU16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/**
 * @brief 从缓冲区按小端读出 32 位数值
 * @param p 源缓冲区，至少 4 字节
 * @return 拼装出的 32 位数值（用于版本号、跟焦器位置等字段）
 */
static inline uint32_t getU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ============================ 构造 / 初始化 ============================ */

/**
 * @brief 默认构造函数
 *
 * 依次完成三件事：
 *  1. 用 *this 初始化 5 个子对象（set / move / get / focusMotor / camera），
 *     它们各自保存一份对本对象的引用，所以只能在构造时绑定；
 *  2. 初始化内部状态：应答缓冲清零、无推送回调、等待应答超时 = 2000ms；
 *  3. 调用 init() 载入默认协议配置（绝对位置控制、三轴有效、不加密、必须应答）。
 *
 * @note 对象约 1.1KB，内部全是固定缓冲，建议定义为全局/静态变量，不要放在任务栈上。
 * @note 构造完成后还没有 CAN 端口，需要再调用 attach()，或改用带端口的构造函数。
 */
RoninS::RoninS()
    : set(*this),
      move(*this),
      get(*this),
      focusMotor(*this),
      camera(*this),
      m_driver(),
      m_positionCtrlByte(0x00),
      m_speedCtrlByte(0x00),
      m_enc(0x00),
      m_cmdType(0x00),
      m_timeoutMs(RS_DEFAULT_TIMEOUT_MS),
      m_combine(),
      m_parser(),
      m_pendingCount(0),
      m_pushCb(nullptr),
      m_pushUser(nullptr)
{
    for (uint8_t i = 0; i < RS_PENDING_MAX; ++i)
        m_pending[i] = RsMessage();

    init();
}

/**
 * @brief 带 CAN 驱动的构造函数
 * @param driver 平台 CAN 驱动接口（函数指针表），内部按值拷贝一份
 * @note 只绑定、不调用 driver.init() 初始化硬件（构造函数没有返回值，失败也没法上报）；
 *       需要初始化硬件请改用 attach()，它会调用 init() 并把结果返回给你。
 * @note explicit：禁止 `RoninS ronin = driver;` 这种隐式转换，只能写 `RoninS ronin(driver);`。
 */
RoninS::RoninS(const rs_can_driver_t &driver)
    : RoninS()
{
    m_driver = driver;
}

/**
 * @brief 初始化协议配置（对应原 C 版的 dji_rs_sdk_init()）
 *
 * 只改软件配置、不发送任何 CAN 帧，调用一次即可：
 *  - 位置控制标志：绝对位置控制（RS_BIT0 = 1），三轴全部有效（RS_BIT1 ~ RS_BIT3 = 0）；
 *  - 速度控制标志：不接管速度控制杆（RS_BIT7 = 0），移动速度不考虑镜头焦距（RS_BIT3 = 1）；
 *  - 帧头配置：ENC = 不加密，CmdType = 必须应答 + 命令帧。
 *
 * @note 重复调用会把上面三项恢复成默认值，之前用 set.axis_valid()、
 *       set.moveJoint_mode()、set.moveSpeed_mode() 改过的设置会被清掉。
 */
void RoninS::init()
{
    m_positionCtrlByte = 0x00;
    m_positionCtrlByte |= (uint8_t)RS_BIT0; /* 默认绝对位置控制 */
    m_speedCtrlByte = 0x00;
    m_speedCtrlByte |= (uint8_t)RS_BIT3; /* 默认移动速度不考虑相机焦距影响 */

    m_enc = (uint8_t)RS_ENC_NONE; /* 不加密 */
    /* CmdType[4:0] = 应答类型（必须应答），CmdType[5] = 帧类型（命令帧） */
    m_cmdType = (uint8_t)(((uint8_t)RS_RESP_MUST & 0x1F) |
                          (((uint8_t)RS_FRAME_COMMAND << 5) & 0x20));
}

/**
 * @brief 绑定 CAN 驱动并初始化它
 * @param driver 平台 CAN 驱动接口（send / receive / millis 必须实现）
 * @return true = 绑定并初始化成功；false = 缺少必要回调，或 driver.init() 返回失败
 * @note 驱动表按值拷贝，所以 driver 这个局部变量用完可以丢弃；
 *       但 driver.ctx 指向的资源必须比 RoninS 活得久。
 * @note attach() 会调用 driver.init()（如果实现了），平台上不需要再单独初始化一次。
 * @note 想换驱动时再调用一次 attach()，或先 detach() 再绑定新的。
 */
bool RoninS::attach(const rs_can_driver_t &driver)
{
    /* send / receive / millis 是 SDK 正常工作的最低要求 */
    if (driver.send == nullptr || driver.receive == nullptr || driver.millis == nullptr)
        return false;

    m_driver = driver; /* 拷贝驱动表，之后不再依赖调用方的那个结构体 */

    if (m_driver.init != nullptr && !m_driver.init(m_driver.ctx))
    {
        detach();
        return false;
    }
    return true;
}

/**
 * @brief 解绑 CAN 驱动：调用 driver.deinit() 并清空内部驱动表
 * @note 调用后 isAttached() 变为 false，未重新 attach() 之前所有命令都会返回失败。
 */
void RoninS::detach()
{
    if (m_driver.deinit != nullptr)
        m_driver.deinit(m_driver.ctx);

    m_driver.init = nullptr;
    m_driver.deinit = nullptr;
    m_driver.send = nullptr;
    m_driver.receive = nullptr;
    m_driver.millis = nullptr;
    m_driver.delay_ms = nullptr;
    m_driver.ctx = nullptr;
}

/**
 * @brief 注册参数推送 / 校准状态推送的回调
 * @param cb   回调函数指针，传 nullptr 表示取消注册
 * @param user 用户自定义指针，回调时原样传回（例如放 this、队列句柄、统计变量）
 * @note 云台主动推送 0x0E/0x08（参数）与 0x0E/0x10（校准状态）时触发；
 *       回调是在"调用 poll() 或等待应答的那个任务"里执行的，不要在回调里长时间阻塞。
 * @note 回调收到的 RsMessage 指向 SDK 内部缓冲，需要保留的数据请立刻拷贝出来。
 */
void RoninS::setPushCallback(RsPushCallback cb, void *user)
{
    m_pushCb = cb;
    m_pushUser = user;
}

/**
 * @brief 处理一次接收：取一帧 CAN 数据 -> 组帧 -> 分发（推送回调 / 应答缓冲）
 * @param timeout_ms 最多等待多少毫秒，0 表示非阻塞、立即返回
 * @return true = 收到了一帧 CAN 数据（不代表一定组成了完整协议帧）；false = 超时无数据
 * @note 需要应答的接口（connect()、move.Joint() 等）内部已经边收边等，
 *       所以 poll() 主要用于空闲时接收云台主动推送的参数帧，
 *       典型用法是在主循环里周期调用 ronin.poll(1000)。
 * @note 只关心两种 CAN ID：0x222（云台应答）与 0x223（自环/推送）。
 */
bool RoninS::poll(uint32_t timeout_ms)
{
    return serviceReceive(timeout_ms);
}

/**
 * @brief 连接云台：查询模块 0x00000001 的版本号，用它确认链路是否通
 * @return true = 收到 0x00（成功）应答，说明接线、波特率、云台供电都正常；
 *         false = 超时或返回错误（最多阻塞 2s，超时时间可用 setResponseTimeout() 修改）
 * @note 协议 2.3.4.10，内部调用 get.module_version()，版本号本身不使用。
 */
bool RoninS::connect()
{
    uint32_t version = 0;
    /* 通过查询模块 0x00000001 的版本号确认链路是否可用 */
    return get.module_version(0x00000001u, &version) == EXECUTION_SUCCESSFUL;
}

/**
 * @brief 断开云台：发送"禁用参数推送"命令（协议 2.3.4.8，ctrl = 0x02）
 * @return true = 云台返回 0x00（成功）；false = 超时或失败
 * @note 只是让云台停止主动推送数据，不会关闭 CAN 硬件，之后仍可以继续发控制命令。
 */
bool RoninS::disconnect()
{
    /* 0x02 = 禁用参数推送 */
    return set.param_push(0x02) == EXECUTION_SUCCESSFUL;
}

/* ============================ Set：云台配置 ============================ */

/**
 * @brief 设置位置控制中各轴是否有效（只改软件标志，不立即发命令）
 * @param axis  YAW / ROLL / PITCH
 * @param valid true = 该轴参与位置控制；false = 该轴无效（云台不响应这个轴）
 * @return true = 设置成功；false = 轴类型非法
 * @note 对应位置控制标志字节的 RS_BIT1(Yaw) / RS_BIT2(Roll) / RS_BIT3(Pitch)，
 *       这三位是"0 有效、1 无效"，所以 valid = true 时执行的是清位操作。
 * @note 设置结果会在下一次 move.Joint() / move.JointNoWait() 时随帧发出。
 */
bool RoninS::Set::axis_valid(enum AxisType axis, bool valid)
{
    switch (axis)
    {
    case YAW: /* RS_BIT1：0 = Yaw 有效，1 = Yaw 无效 */
        if (valid)
            m_owner.m_positionCtrlByte &= (uint8_t)(~RS_BIT1);
        else
            m_owner.m_positionCtrlByte |= (uint8_t)RS_BIT1;
        break;
    case ROLL: /* RS_BIT2：0 = Roll 有效，1 = Roll 无效 */
        if (valid)
            m_owner.m_positionCtrlByte &= (uint8_t)(~RS_BIT2);
        else
            m_owner.m_positionCtrlByte |= (uint8_t)RS_BIT2;
        break;
    case PITCH: /* RS_BIT3：0 = Pitch 有效，1 = Pitch 无效 */
        if (valid)
            m_owner.m_positionCtrlByte &= (uint8_t)(~RS_BIT3);
        else
            m_owner.m_positionCtrlByte |= (uint8_t)RS_BIT3;
        break;
    default:
        return false;
    }
    return true;
}

/**
 * @brief 设置位置控制模式（只改软件标志，不立即发命令）
 * @param type ABSOLUTE_CONTROL = 绝对角度控制；INCREMENTAL_CONTROL = 相对当前角度的增量控制
 * @return 恒为 true
 * @note 对应位置控制标志字节的 RS_BIT0（1 = 绝对，0 = 增量），默认由 init() 设为绝对模式。
 * @note 增量模式下，move.Joint() 传入的角度表示"要在当前基础上转多少度"。
 */
bool RoninS::Set::moveJoint_mode(enum MoveMode type)
{
    if (type == ABSOLUTE_CONTROL)
        m_owner.m_positionCtrlByte |= (uint8_t)RS_BIT0; /* 绝对位置模式 */
    else
        m_owner.m_positionCtrlByte &= (uint8_t)(~RS_BIT0); /* 增量（相对）位置模式 */
    return true;
}

/**
 * @brief 设置速度控制杆与"移动速度是否考虑镜头焦距"（只改软件标志）
 * @param speed_type sENABLED = 接管云台速度控制杆；sDISABLED = 释放速度控制杆
 * @param focal_mode fENABLED = 移动速度考虑相机焦距影响；fDISABLED = 不考虑
 * @return 恒为 true
 * @note 对应速度控制标志字节的 RS_BIT7（速度控制杆）与 RS_BIT3（焦距影响，1 = 不考虑）。
 * @note 是否接管速度控制杆会影响速度类命令与手动推杆的关系，设置完成后在下一条速度命令生效。
 */
bool RoninS::Set::moveSpeed_mode(enum SpeedControl speed_type, enum FocalControl focal_mode)
{
    if (speed_type == sENABLED)
        m_owner.m_speedCtrlByte |= (uint8_t)RS_BIT7; /* 接管云台速度控制 */
    else
        m_owner.m_speedCtrlByte &= (uint8_t)(~RS_BIT7); /* 释放速度控制 */

    if (focal_mode == fENABLED)
        m_owner.m_speedCtrlByte &= (uint8_t)(~RS_BIT3); /* 移动速度考虑相机焦距影响 */
    else
        m_owner.m_speedCtrlByte |= (uint8_t)RS_BIT3; /* 不考虑相机焦距影响 */
    return true;
}

/**
 * @brief 2.3.4.4 设置手持云台限位角度（发送并等待应答）
 * @param yaw_max / yaw_min     Yaw 轴上限 / 下限，单位 1°，范围 0 ~ 179
 * @param pitch_max / pitch_min Pitch 轴上限 / 下限，单位 1°，范围 0 ~ 179
 * @param roll_max / roll_min   Roll 轴上限 / 下限，单位 1°，范围 0 ~ 179
 * @return 云台返回码（0x00 = EXECUTION_SUCCESSFUL 成功），或 RS_TIMEOUT / RS_ERROR
 * @note 数据段第一个字节固定 0x01（表示"设置限位"），后面按
 *       pitch_max、pitch_min、yaw_max、yaw_min、roll_max、roll_min 排列，
 *       协议里是 Pitch 在前、Yaw 在后，与函数参数顺序不同，内部已按协议重排。
 * @note 任一参数超过 179 时直接返回 RS_ERROR，不发送任何帧。
 */
uint8_t RoninS::Set::limit_angle(uint8_t yaw_max, uint8_t yaw_min,
                                 uint8_t pitch_max, uint8_t pitch_min,
                                 uint8_t roll_max, uint8_t roll_min)
{
    uint8_t payload[7] = {0x01, pitch_max, pitch_min, yaw_max, yaw_min, roll_max, roll_min};
    uint8_t resp[8];
    uint16_t resp_len = 0;

    if (pitch_max > 179 || pitch_min > 179 ||
        yaw_max > 179 || yaw_min > 179 ||
        roll_max > 179 || roll_min > 179)
    {
        return RS_ERROR;
    }

    /* 2.3.4.4 设置限位角度 */
    return m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x03, payload, sizeof(payload),
                               resp, sizeof(resp), &resp_len);
}

/**
 * @brief 2.3.4.6 设置手持云台三轴电机力度（发送并等待应答）
 * @param yaw / pitch / roll 三轴电机力度，范围 0 ~ 100（数值越大云台越"硬"）
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 数据段为 0x01(设置标志) + pitch + roll + yaw，协议里顺序是 Pitch、Roll、Yaw。
 * @note 任一参数超过 100 直接返回 RS_ERROR，不发帧。
 * @note 力度太小云台容易"发飘"，太大画面容易抖，一般 30 ~ 70 比较合适。
 */
uint8_t RoninS::Set::motor_stiffness(uint8_t yaw, uint8_t pitch, uint8_t roll)
{
    uint8_t payload[4] = {0x01, pitch, roll, yaw};
    uint8_t resp[8];
    uint16_t resp_len = 0;

    if (pitch > 100 || roll > 100 || yaw > 100)
        return RS_ERROR;

    /* 2.3.4.6 设置电机力度 */
    return m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x05, payload, sizeof(payload),
                               resp, sizeof(resp), &resp_len);
}

/**
 * @brief 2.3.4.8 参数推送设置（发送并等待应答）
 * @param ctrl 0x00 = 无操作，0x01 = 使能参数推送，0x02 = 禁用参数推送
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 使能后云台会周期推送 0x0E/0x08 参数帧（角度 / 限位 / 电机力度），
 *       配合 setPushCallback() 注册的回调使用；disconnect() 内部用的就是 ctrl = 0x02。
 */
uint8_t RoninS::Set::param_push(uint8_t ctrl)
{
    uint8_t resp[8];
    uint16_t resp_len = 0;

    /* 2.3.4.8 参数推送设置：0x00 无操作 / 0x01 使能 / 0x02 禁用 */
    return m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x07, &ctrl, 1,
                               resp, sizeof(resp), &resp_len);
}

/**
 * @brief 2.3.4.14 设置工作模式与云台方向（发送并等待应答）
 * @param work_mode   工作模式，GIMBAL_MODE_KEEP(0xFE) = 保持当前工作模式不变
 * @param orientation 云台方向，取值见 enum GimbalOrientation（横拍 / 竖拍 / 切换 / 恢复默认）
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 数据段就是两个字节：{work_mode, orientation}。
 */
uint8_t RoninS::Set::work_mode(uint8_t work_mode, uint8_t orientation)
{
    uint8_t payload[2] = {work_mode, orientation};
    uint8_t resp[8];
    uint16_t resp_len = 0;

    /* 2.3.4.14 设置工作模式与云台方向 */
    return m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x0D, payload, sizeof(payload),
                               resp, sizeof(resp), &resp_len);
}

/**
 * @brief 2.3.4.15 云台回中 / 自拍（发送并等待应答）
 * @param action CENTER_CMD(0x01) = 云台回中；SELFIE_CMD(0x02) = 自拍
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 数据段固定为 {0xFE, action}，0xFE 表示"保持工作模式不变，只执行动作"。
 * @note 回中时云台会自己转动，使用前请确认周围没有遮挡物。
 */
uint8_t RoninS::Set::back_center(uint8_t action)
{
    uint8_t payload[2] = {0xFE, action};
    uint8_t resp[8];
    uint16_t resp_len = 0;

    /* 2.3.4.15 回中 / 自拍 */
    return m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x0E, payload, sizeof(payload),
                               resp, sizeof(resp), &resp_len);
}

/**
 * @brief 2.3.4.15 设置云台跟随模式（发送并等待应答）
 * @param follow_mode FOLLOW_MODE_LOCKED(0x00) 锁定 / FOLLOW_MODE_YAW_FOLLOW(0x02) Yaw 跟随 /
 *                    FOLLOW_MODE_SPORT(0x03) 运动模式
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 数据段为 {follow_mode, 0x00}，第二个字节固定 0（不触发回中/自拍）。
 * @note 与 back_center() 使用同一条命令 ID 0x0E，靠数据段内容区分具体动作。
 */
uint8_t RoninS::Set::follow_mode(uint8_t follow_mode)
{
    uint8_t payload[2] = {follow_mode, 0x00};
    uint8_t resp[8];
    uint16_t resp_len = 0;

    /* 2.3.4.15 设置跟随模式 */
    return m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x0E, payload, sizeof(payload),
                               resp, sizeof(resp), &resp_len);
}

/**
 * @brief 2.3.4.16 设置自动校准（无应答，只发送）
 * @param enable 0 = 关闭校准，1 = 开启校准
 * @param type   校准类型（云台自校准 / 电机校准等，取值见协议文档）
 * @return EXECUTION_SUCCESSFUL = 帧已成功发出；RS_ERROR = 发送失败
 * @note 云台不会回应这条命令，所以只能判断"帧有没有发出去"，无法判断校准是否成功。
 * @note 数据段 3 字节：0x00、0x01、((type << 1) | (enable & 0x01))，最后一字节把类型和开关打包。
 * @note 校准时云台会大幅度转动，务必先取下相机、让出活动空间。
 */
uint8_t RoninS::Set::auto_calibration(uint8_t enable, uint8_t type)
{
    uint8_t payload[3] = {0x00, 0x01, (uint8_t)((type << 1) | (enable & 0x01))};

    /* 2.3.4.16 自动校准：无应答，只判断是否发送成功 */
    return m_owner.sendCmd(RS_CMD_SET_GIMBAL, 0x0F, payload, sizeof(payload), nullptr)
               ? (uint8_t)EXECUTION_SUCCESSFUL
               : (uint8_t)RS_ERROR;
}

/**
 * @brief 2.3.4.18 智能跟随（无应答，只发送）
 * @param cmd 智能跟随命令，当前协议仅支持 0x03
 * @return EXECUTION_SUCCESSFUL = 帧已成功发出；RS_ERROR = 命令非法或发送失败
 * @note 传入其它取值会直接返回 RS_ERROR，不发送任何帧。
 */
uint8_t RoninS::Set::smart_follow(uint8_t cmd)
{
    if (cmd != 0x03)
        return RS_ERROR; /* 当前协议仅支持 0x03 */

    /* 2.3.4.18 智能跟随：无应答 */
    return m_owner.sendCmd(RS_CMD_SET_GIMBAL, 0x11, &cmd, 1, nullptr)
               ? (uint8_t)EXECUTION_SUCCESSFUL
               : (uint8_t)RS_ERROR;
}

/**
 * @brief 2.3.4.13 设置用户参数（TLV 格式，发送并等待应答）
 * @param tlv_id     参数 ID
 * @param tlv_length 参数数据长度，最大 32 字节
 * @param tlv_data   参数数据指针（tlv_length 为 0 时可以传 nullptr）
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 数据段 = tlv_id(1B) + tlv_length(1B) + 数据；
 *       长度超过 32 或数据指针为空时直接返回 RS_ERROR。
 * @note 载荷最长 34 字节，协议帧总长会超过 8 字节，由发送函数自动拆成多条 CAN 帧。
 */
uint8_t RoninS::Set::user_param(uint8_t tlv_id, uint8_t tlv_length, const uint8_t *tlv_data)
{
    uint8_t payload[34];
    uint8_t resp[64];
    uint16_t resp_len = 0;

    if (tlv_length > 32)
        return RS_ERROR;
    if (tlv_length > 0 && tlv_data == nullptr)
        return RS_ERROR;

    payload[0] = tlv_id;
    payload[1] = tlv_length;
    for (uint8_t i = 0; i < tlv_length; ++i)
        payload[2 + i] = tlv_data[i];

    /* 2.3.4.13 设置用户参数（TLV） */
    return m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x0C, payload, (uint16_t)(2 + tlv_length),
                               resp, sizeof(resp), &resp_len);
}

/* ============================ Move：云台运动 ============================ */

/**
 * @brief 2.3.4.1 手持云台位置控制（发送并等待云台应答）
 * @param yaw    目标偏航角，单位 度，范围 -180.0 ~ 180.0
 * @param pitch  目标俯仰角，单位 度，范围 -180.0 ~ 180.0
 * @param roll   目标横滚角，单位 度，范围 -180.0 ~ 180.0
 * @param time_s 运动时间，单位 秒，范围 0.1 ~ 25.5
 * @return 云台返回码（0x00 = EXECUTION_SUCCESSFUL 成功），或 RS_TIMEOUT / RS_ERROR
 * @note 参数顺序是 (yaw, pitch, roll, time_s)，而协议帧里的顺序是 Yaw、Roll、Pitch，
 *       内部已做单位换算与字段重排，调用方不用关心。
 * @note 内部先调用 JointNoWait() 发帧，再用本帧序列号等应答，因此不会与上一条命令的应答混淆。
 * @note 要做 50ms 周期的高频控制时改用 JointNoWait()，避免每条命令都阻塞等应答。
 */
uint8_t RoninS::Move::Joint(float yaw, float pitch, float roll, float time_s)
{
    uint8_t seq[2] = {0, 0};

    /* 先发送位置控制命令，再用序列号等待本次应答 */
    if (!JointNoWait(yaw, pitch, roll, time_s, seq))
        return RS_ERROR;

    return m_owner.waitResponse(seq, RS_CMD_SET_GIMBAL, 0x00, nullptr, 0, nullptr,
                                m_owner.m_timeoutMs);
}

/**
 * @brief 2.3.4.1 手持云台位置控制（只发送，不等待应答）
 * @param yaw / pitch / roll 目标角度，单位 度，范围 -180.0 ~ 180.0
 * @param time_s             运动时间，单位 秒，范围 0.1 ~ 25.5
 * @param seq                可选输出参数，回填本帧的 2 字节序列号（想自己等应答时传入）
 * @return true = 已成功发送；false = 参数越界或发送失败
 * @note 角度会先乘 10 换算成协议单位 0.1°，时间换算成 0.1s；
 *       任意一项越界都直接返回 false，不会发送任何帧。
 * @note 数据段 8 字节：Yaw(2) + Roll(2) + Pitch(2) + 位置控制标志(1) + 运动时间(1)，
 *       标志字节由 set.moveJoint_mode() 与 set.axis_valid() 配置。
 */
bool RoninS::Move::JointNoWait(float yaw, float pitch, float roll, float time_s, uint8_t *seq)
{
    int16_t yaw_x10 = (int16_t)(yaw * 10.0f);     /* 角度换算为 0.1° 单位 */
    int16_t pitch_x10 = (int16_t)(pitch * 10.0f);
    int16_t roll_x10 = (int16_t)(roll * 10.0f);
    uint16_t time_x10 = (uint16_t)(time_s * 10.0f); /* 时间换算为 0.1s 单位 */
    uint8_t payload[8];

    /* 2.3.4.1 位置控制：yaw -180.0 ~ 180.0°，pitch/roll 同，时间 0.1 ~ 25.5s */
    if (!(yaw_x10 >= -1800 && yaw_x10 <= 1800 &&
          pitch_x10 >= -1800 && pitch_x10 <= 1800 &&
          roll_x10 >= -1800 && roll_x10 <= 1800 &&
          time_x10 >= 1 && time_x10 <= 255))
    {
        return false;
    }

    putU16(&payload[0], (uint16_t)yaw_x10);
    putU16(&payload[2], (uint16_t)roll_x10);
    putU16(&payload[4], (uint16_t)pitch_x10);
    payload[6] = m_owner.m_positionCtrlByte; /* 位置控制标志 */
    payload[7] = (uint8_t)time_x10;          /* 运动时间，单位 0.1s */

    return m_owner.sendCmd(RS_CMD_SET_GIMBAL, 0x00, payload, sizeof(payload), seq);
}

/**
 * @brief 2.3.4.2 手持云台速度控制（发送并等待应答）
 * @param yaw   偏航速度，单位 0.1°/s，范围 0 ~ 3600
 * @param pitch 俯仰速度，单位 0.1°/s，范围 0 ~ 3600
 * @param roll  横滚速度，单位 0.1°/s，范围 0 ~ 3600
 * @return true = 云台返回 0x00（执行成功）；false = 参数越界、等待超时或云台执行失败
 * @note 速度控制一般需要先用 set.moveSpeed_mode(sENABLED, ...) 接管速度控制杆。
 * @note 数据段 7 字节：Yaw(2) + Roll(2) + Pitch(2) + 速度控制标志(1)。
 * @note 本接口只返回 true/false，超时(RS_TIMEOUT)与云台执行失败都会表现为 false。
 */
bool RoninS::Move::Speed(int16_t yaw, int16_t pitch, int16_t roll)
{
    uint8_t payload[7];
    uint8_t resp[8];
    uint16_t resp_len = 0;

    /* 2.3.4.2 速度控制：各轴 0 ~ 3600（0.1°/s） */
    if (!(yaw >= 0 && yaw <= 3600 &&
          pitch >= 0 && pitch <= 3600 &&
          roll >= 0 && roll <= 3600))
    {
        return false;
    }

    putU16(&payload[0], (uint16_t)yaw);
    putU16(&payload[2], (uint16_t)roll);
    putU16(&payload[4], (uint16_t)pitch);
    payload[6] = m_owner.m_speedCtrlByte; /* 速度控制标志 */

    return m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x01, payload, sizeof(payload),
                               resp, sizeof(resp), &resp_len) == EXECUTION_SUCCESSFUL;
}

/**
 * @brief 2.3.4.11 摇杆控制（无应答，只发送）
 * @param yaw / pitch / roll 三轴速度，单位 0.1°/s，可正可负（负号表示反方向）
 * @return 无
 * @note 数据段 7 字节：0x01(摇杆遥控器标志) + Pitch(2) + Roll(2) + Yaw(2)。
 * @note 这是"推杆"语义：需要周期性连续发送（例如每 50ms 一次）才能维持运动，
 *       停止发送后云台会逐渐回到原来的状态。
 */
void RoninS::Move::Joystick(int16_t yaw, int16_t pitch, int16_t roll)
{
    uint8_t payload[7];

    payload[0] = 0x01; /* 摇杆遥控器 */
    putU16(&payload[1], (uint16_t)pitch);
    putU16(&payload[3], (uint16_t)roll);
    putU16(&payload[5], (uint16_t)yaw);

    /* 2.3.4.11 摇杆控制：无应答 */
    m_owner.sendCmd(RS_CMD_SET_GIMBAL, 0x0A, payload, sizeof(payload), nullptr);
}

/**
 * @brief 2.3.4.11 拨轮控制（无应答，只发送）
 * @param dial_speed 拨轮速度，单位 0.1°/s，可正可负
 * @return 无
 * @note 数据段 3 字节：0x02(拨轮遥控器标志) + 速度(2, 小端)。
 * @note 与 Joystick() 共用命令 ID 0x0A，靠数据段第一字节区分是摇杆还是拨轮。
 */
void RoninS::Move::Dial(int16_t dial_speed)
{
    uint8_t payload[3];

    payload[0] = 0x02; /* 拨轮遥控器 */
    putU16(&payload[1], (uint16_t)dial_speed);

    /* 2.3.4.11 拨轮控制：无应答 */
    m_owner.sendCmd(RS_CMD_SET_GIMBAL, 0x0A, payload, sizeof(payload), nullptr);
}

/* ============================ Get：云台查询 ============================ */

/**
 * @brief 2.3.4.3 获取云台角度（发送并等待应答）
 * @param DataType GIMBAL_ATTITUDE_ANGLE(0x01) 姿态角 / GIMBAL_JOINT_ANGLE(0x02) 关节角
 * @param yaw / pitch / roll 输出参数，单位 0.1°（例如 900 表示 90.0°）；
 *                           不关心的轴可以传 nullptr 跳过
 * @return 云台返回码（0x00 = EXECUTION_SUCCESSFUL），或 RS_TIMEOUT / RS_ERROR
 * @note 应答格式：返回码(1) + 角度类型(1) + Yaw(2) + Roll(2) + Pitch(2)。
 *       应答里的角度类型必须与请求一致，否则返回 RS_ERROR。
 * @note 应答长度不足 8 字节时同样返回 RS_ERROR，不会把半截数据写进输出参数。
 */
uint8_t RoninS::Get::gimbal_angle(uint8_t DataType, int16_t *yaw, int16_t *pitch, int16_t *roll)
{
    uint8_t resp[16];
    uint16_t resp_len = 0;
    uint8_t code;

    /* 2.3.4.3 获取云台角度（姿态角 / 关节角） */
    code = m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x02, &DataType, 1,
                               resp, sizeof(resp), &resp_len);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    /* 应答至少 8 字节：返回码 + 角度类型 + 3 组角度 */
    if (resp_len < 8)
        return RS_ERROR;
    if (resp[1] == 0x00 || resp[1] != DataType)
        return RS_ERROR; /* 应答类型必须与请求一致 */

    if (yaw != nullptr)
        *yaw = (int16_t)getU16(&resp[2]);
    if (roll != nullptr)
        *roll = (int16_t)getU16(&resp[4]);
    if (pitch != nullptr)
        *pitch = (int16_t)getU16(&resp[6]);
    return code;
}

/**
 * @brief 2.3.4.5 获取手持云台限位角度（发送并等待应答）
 * @param yaw_max / yaw_min     Yaw 轴上限 / 下限，单位 1°
 * @param pitch_max / pitch_min Pitch 轴上限 / 下限，单位 1°
 * @param roll_max / roll_min   Roll 轴上限 / 下限，单位 1°
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 每个输出指针都可以传 nullptr，表示"这一项不需要"。
 * @note 应答格式：返回码 + pitch_max、pitch_min、yaw_max、yaw_min、roll_max、roll_min，
 *       协议里先是 Pitch 再是 Yaw，内部已按协议解析。
 */
uint8_t RoninS::Get::limit_angle(uint8_t *yaw_max, uint8_t *yaw_min,
                                 uint8_t *pitch_max, uint8_t *pitch_min,
                                 uint8_t *roll_max, uint8_t *roll_min)
{
    uint8_t ctrl = 0x01;
    uint8_t resp[16];
    uint16_t resp_len = 0;
    uint8_t code;

    /* 2.3.4.5 获取限位角度 */
    code = m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x04, &ctrl, 1,
                               resp, sizeof(resp), &resp_len);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 7)
        return RS_ERROR;

    if (pitch_max != nullptr)
        *pitch_max = resp[1];
    if (pitch_min != nullptr)
        *pitch_min = resp[2];
    if (yaw_max != nullptr)
        *yaw_max = resp[3];
    if (yaw_min != nullptr)
        *yaw_min = resp[4];
    if (roll_max != nullptr)
        *roll_max = resp[5];
    if (roll_min != nullptr)
        *roll_min = resp[6];
    return code;
}

/**
 * @brief 2.3.4.7 获取手持云台三轴电机力度（发送并等待应答）
 * @param yaw / pitch / roll 输出参数，范围 0 ~ 100；不需要的可以传 nullptr
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 应答格式：返回码 + pitch + yaw + roll。注意这条查询应答里的顺序是
 *       Pitch、Yaw、Roll，与设置命令 set.motor_stiffness(pitch, roll, yaw) 不同。
 */
uint8_t RoninS::Get::motor_stiffness(uint8_t *yaw, uint8_t *pitch, uint8_t *roll)
{
    uint8_t ctrl = 0x01;
    uint8_t resp[16];
    uint16_t resp_len = 0;
    uint8_t code;

    /* 2.3.4.7 获取电机力度 */
    code = m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x06, &ctrl, 1,
                               resp, sizeof(resp), &resp_len);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 4)
        return RS_ERROR;

    if (pitch != nullptr)
        *pitch = resp[1];
    if (yaw != nullptr)
        *yaw = resp[2];
    if (roll != nullptr)
        *roll = resp[3];
    return code;
}

/**
 * @brief 2.3.4.10 获取模块版本号（发送并等待应答）
 * @param device_id 设备 ID，云台模块通常传 0x00000001
 * @param version   输出参数，0xAABBCCDD 表示版本 AA.BB.CC.DD；可以传 nullptr 只看是否成功
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note connect() 就是用这条命令（device_id = 0x00000001）来判断链路是否连通的。
 * @note 应答长度不足 9 字节时返回 RS_ERROR。
 */
uint8_t RoninS::Get::module_version(uint32_t device_id, uint32_t *version)
{
    uint8_t payload[4];
    uint8_t resp[16];
    uint16_t resp_len = 0;
    uint8_t code;

    payload[0] = (uint8_t)(device_id & 0xFF);
    payload[1] = (uint8_t)((device_id >> 8) & 0xFF);
    payload[2] = (uint8_t)((device_id >> 16) & 0xFF);
    payload[3] = (uint8_t)((device_id >> 24) & 0xFF);

    /* 2.3.4.10 获取模块版本号 */
    code = m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x09, payload, sizeof(payload),
                               resp, sizeof(resp), &resp_len);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 9)
        return RS_ERROR;

    if (version != nullptr)
        *version = getU32(&resp[5]);
    return code;
}

/**
 * @brief 2.3.4.12 获取用户参数（TLV 格式，发送并等待应答）
 * @param read_ids   要读取的参数 ID 数组（不能为空）
 * @param id_num     要读取的参数个数（不能为 0）
 * @param tlv_buffer 输出缓冲，用来接收应答中的 TLV 数据
 * @param tlv_length 输出参数，返回 TLV 数据的实际字节数；可以传 nullptr
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 应答格式：返回码 + TLV 数据，所以实际数据长度 = 应答长度 - 1。
 * @note 本函数不检查 tlv_buffer 的容量，调用方需保证足够大（单次应答不会超过 63 字节）。
 */
uint8_t RoninS::Get::user_param(uint8_t *read_ids, uint8_t id_num, uint8_t *tlv_buffer, uint16_t *tlv_length)
{
    uint8_t resp[64];
    uint16_t resp_len = 0;
    uint8_t code;

    if (read_ids == nullptr || id_num == 0)
        return RS_ERROR;

    /* 2.3.4.12 获取用户参数（TLV） */
    code = m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x0B, read_ids, id_num,
                               resp, sizeof(resp), &resp_len);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 1)
        return RS_ERROR;

    if (tlv_length != nullptr)
        *tlv_length = (uint16_t)(resp_len - 1);
    if (tlv_buffer != nullptr)
    {
        for (uint16_t i = 1; i < resp_len; ++i)
            tlv_buffer[i - 1] = resp[i];
    }
    return code;
}

/* ======================== FocusMotor：跟焦器电机 ======================== */

/**
 * @brief 2.3.4.19 跟焦器电机位置控制（无应答，只发送）
 * @param position 目标位置（有符号 16 位，内部按小端发送）
 * @return 无
 * @note 数据段 5 字节：0x01(位置控制) + 0x00 + 0x02 + 位置(2, 小端)。
 * @note 与 calibrate()、get_position() 共用命令 ID 0x12，靠数据段第一字节区分操作类型。
 */
void RoninS::FocusMotor::move_position(int16_t position)
{
    uint8_t payload[5] = {0x01, 0x00, 0x02, 0x00, 0x00};

    putU16(&payload[3], (uint16_t)position);

    /* 2.3.4.19 跟焦器电机位置控制：无应答 */
    m_owner.sendCmd(RS_CMD_SET_GIMBAL, 0x12, payload, sizeof(payload), nullptr);
}

/**
 * @brief 2.3.4.19 跟焦器电机校准（发送并等待应答）
 * @param MotorType 电机类型
 * @param cmd       校准命令（开始 / 停止等，取值见协议文档）
 * @return 云台返回码（0x00 = EXECUTION_SUCCESSFUL 成功），或 RS_TIMEOUT / RS_ERROR
 * @note 数据段 3 字节：0x02(校准) + MotorType + cmd。
 */
uint8_t RoninS::FocusMotor::calibrate(uint8_t MotorType, uint8_t cmd)
{
    uint8_t payload[3] = {0x02, MotorType, cmd};
    uint8_t resp[16];
    uint16_t resp_len = 0;

    /* 2.3.4.19 跟焦器电机校准 */
    return m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x12, payload, sizeof(payload),
                               resp, sizeof(resp), &resp_len);
}

/**
 * @brief 2.3.4.19 获取跟焦器电机位置（发送并等待应答）
 * @param type        电机类型
 * @param calib_state 输出参数，校准状态；可以传 nullptr 跳过
 * @param position    输出参数，当前位置；可以传 nullptr 跳过
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 请求数据 2 字节：0x15(读取位置) + type。
 * @note 应答格式：返回码 + ... + 校准状态(第 4 字节) + 位置(第 5 ~ 8 字节，小端)；
 *       应答长度不足 8 字节时返回 RS_ERROR。
 */
uint8_t RoninS::FocusMotor::get_position(uint8_t type, uint8_t *calib_state, uint32_t *position)
{
    uint8_t payload[2] = {0x15, type};
    uint8_t resp[16];
    uint16_t resp_len = 0;
    uint8_t code;

    /* 2.3.4.19 获取跟焦器电机位置 */
    code = m_owner.sendAndWait(RS_CMD_SET_GIMBAL, 0x12, payload, sizeof(payload),
                               resp, sizeof(resp), &resp_len);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 8)
        return RS_ERROR;

    if (calib_state != nullptr)
        *calib_state = resp[3];
    if (position != nullptr)
        *position = getU32(&resp[4]);
    return code;
}

/* ============================ Camera：相机 ============================ */

/**
 * @brief 2.3.5.1 相机动作控制（发送并等待应答）
 * @param action 相机动作，取值见 enum CameraAction（拍照、录像启停、中心对焦等）
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 使用相机命令集 0x0D、命令 ID 0x00，数据段为 2 字节小端动作码。
 */
uint8_t RoninS::Camera::action(uint16_t action)
{
    uint8_t payload[2];
    uint8_t resp[8];
    uint16_t resp_len = 0;

    putU16(payload, action);

    /* 2.3.5.1 相机动作控制 */
    return m_owner.sendAndWait(RS_CMD_SET_CAMERA, 0x00, payload, sizeof(payload),
                               resp, sizeof(resp), &resp_len);
}

/**
 * @brief 2.3.5 获取相机状态（发送并等待应答）
 * @param status 输出参数，相机状态字节；可以传 nullptr 只看是否成功
 * @return 云台返回码，或 RS_TIMEOUT / RS_ERROR
 * @note 请求数据固定 0x01；应答格式：返回码 + 状态字节，长度不足 2 字节时返回 RS_ERROR。
 */
uint8_t RoninS::Camera::get_status(uint8_t *status)
{
    uint8_t ctrl = 0x01;
    uint8_t resp[8];
    uint16_t resp_len = 0;
    uint8_t code;

    /* 2.3.5 获取相机状态 */
    code = m_owner.sendAndWait(RS_CMD_SET_CAMERA, 0x01, &ctrl, 1,
                               resp, sizeof(resp), &resp_len);
    if (code != EXECUTION_SUCCESSFUL)
        return code;
    if (resp_len < 2)
        return RS_ERROR;

    if (status != nullptr)
        *status = resp[1];
    return code;
}
