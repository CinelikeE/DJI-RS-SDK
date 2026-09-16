/**
 * @file DJI_RS_SDK.h
 * @brief DJI RS 手持云台 SDK 对外接口（C++ 版）
 *
 * 由原 RT-Thread C 版（https://github.com/CinelikeE/DJI-RS-SDK）改写为 C++，
 * 覆盖协议文档 2.3.x 的主要命令：
 *  - 2.3.4.1 ~ 2.3.4.7  位置/速度控制、角度与限位查询、电机力度
 *  - 2.3.4.8 ~ 2.3.4.16 参数推送、模块版本、摇杆/拨轮、用户参数、工作模式、回中/自拍、校准
 *  - 2.3.4.19           跟焦器电机控制
 *  - 2.3.5              相机动作控制
 *
 * 使用方式（三个步骤）：
 * @code
 * static rs_can_driver_t can;       // 1. 平台 CAN 驱动（见 CanInterface.h，用平台 C API 填好）
 * static RoninS ronin;              // 2. SDK 实例（对象约 1KB，建议用全局/静态变量）
 *
 * void app_main(void)
 * {
 *     ronin.attach(can);            // 绑定并调用 can.init()
 *     ronin.init();                 // 初始化协议配置
 *     if (ronin.connect()) {        // 查询模块版本，确认链路
 *         ronin.move.Joint(90.0f, 0.0f, 0.0f, 2.0f);  // Yaw 转到 90°
 *         int16_t yaw = 0, pitch = 0, roll = 0;
 *         ronin.get.gimbal_angle(GIMBAL_ATTITUDE_ANGLE, &yaw, &pitch, &roll);
 *     }
 * }
 * @endcode
 *
 * 约定：
 *  - 需要等待应答的接口返回 uint8_t（云台返回码 0x00 成功，或 RS_TIMEOUT/RS_ERROR）；
 *  - 不需要应答的接口返回 bool（是否发送成功）；
 *  - 角度相关参数/返回值单位一般为 0.1°（0.1°/s、0.1s 分别用于速度和运动时间）；
 *  - 全程不使用 new / malloc，所有缓冲都在对象内部。
 */
#ifndef DJI_RS_SDK_H
#define DJI_RS_SDK_H

#include <stdint.h>

#include "CanInterface.h"
#include "CmdParse.h"
#include "Combine.h"
#include "Response.h"

/* 协议 3.1：PC 侧（本机）CAN 发送 ID = 0x223，接收 ID = 0x222 */
#define RS_CAN_TX_ID 0x223
#define RS_CAN_RX_ID 0x222

/* 命令集 */
#define RS_CMD_SET_GIMBAL 0x0E /* 云台命令集 */
#define RS_CMD_SET_CAMERA 0x0D /* 相机命令集 */

/* 控制标志字节使用的位定义（position_ctrl_byte / speed_ctrl_byte）
 * 注意：加了 RS_ 前缀，避免与 ESP-IDF esp_bit_defs.h 里的 BIT0 ~ BIT31 宏冲突 */
enum Flag
{
    RS_BIT0 = 0x01, /* position: 0 = 增量控制，1 = 绝对控制 */
    RS_BIT1 = 0x02, /* position: Yaw 轴无效标志（1 = 无效） */
    RS_BIT2 = 0x04, /* position: Roll 轴无效标志（1 = 无效） */
    RS_BIT3 = 0x08, /* position: Pitch 轴无效标志；speed: 1 = 移动速度不考虑镜头焦距 */
    RS_BIT4 = 0x10, /* 保留 */
    RS_BIT5 = 0x20, /* 保留 */
    RS_BIT6 = 0x40, /* 保留 */
    RS_BIT7 = 0x80  /* speed: 速度控制杆（1 = 接管云台速度控制） */
};

/* 云台轴类型（位置控制与角度查询通用） */
enum AxisType
{
    YAW = 0,  /* 偏航轴 */
    ROLL = 1, /* 横滚轴 */
    PITCH = 2 /* 俯仰轴 */
};

/* 位置控制模式 */
enum MoveMode
{
    INCREMENTAL_CONTROL = 0, /* 增量（相对）控制：指令给出相对当前角度的增量 */
    ABSOLUTE_CONTROL = 1     /* 绝对控制：指令给出目标绝对角度 */
};

/* 速度控制杆 */
enum SpeedControl
{
    sDISABLED = 0, /* 释放速度控制杆 */
    sENABLED = 1   /* 接管云台速度控制杆 */
};

/* 移动速度是否考虑镜头焦距影响 */
enum FocalControl
{
    fENABLED = 0, /* 移动速度考虑相机焦距影响 */
    fDISABLED = 1 /* 移动速度不考虑相机焦距影响 */
};

/* 2.3.4.3 获取云台信息：角度类型 */
enum GimbalAngleType
{
    GIMBAL_ATTITUDE_ANGLE = 0x01, /* 姿态角（云台整体姿态） */
    GIMBAL_JOINT_ANGLE = 0x02     /* 关节角（各轴电机角度） */
};

/* 2.3.4.14 设置手持云台工作模式 */
enum GimbalWorkMode
{
    GIMBAL_MODE_KEEP = 0xFE /* 保持当前工作模式不变 */
};

/* 2.3.4.14 云台方向（横拍 / 竖拍） */
enum GimbalOrientation
{
    ORIENT_HORIZONTAL_0 = 0x01,   /* 绕 X 轴旋转 0° 的横拍 */
    ORIENT_HORIZONTAL_180 = 0x02, /* 绕 X 轴旋转 180° 的横拍 */
    ORIENT_VERTICAL_90 = 0x03,    /* 绕 X 轴旋转 90° 的竖拍 */
    ORIENT_VERTICAL_NEG90 = 0x04, /* 绕 X 轴旋转 -90° 的竖拍 */
    ORIENT_TOGGLE = 0x05,         /* 横竖拍切换，角度由云台自适应 */
    ORIENT_DEFAULT = 0xFF         /* 恢复默认模式 */
};

/* 2.3.4.15 设置云台跟随模式 */
enum GimbalFollowMode
{
    FOLLOW_MODE_LOCKED = 0x00,     /* 云台锁定模式：Yaw/Roll/Pitch 均锁定 */
    FOLLOW_MODE_YAW_FOLLOW = 0x02, /* Yaw 跟随模式 */
    FOLLOW_MODE_SPORT = 0x03       /* 运动模式 */
};

/* 2.3.4.15 回中/自拍动作 */
enum CenterSelfieCmd
{
    CENTER_CMD = 0x01, /* 云台回中 */
    SELFIE_CMD = 0x02  /* 自拍 */
};

/* 2.3.5.1 第三方相机动作命令 */
enum CameraAction
{
    CAMERA_PHOTO = 0x0001,             /* 拍照 */
    CAMERA_STOP_PHOTO = 0x0002,        /* 停止拍照 */
    CAMERA_RECORD_START = 0x0003,      /* 开始录像 */
    CAMERA_RECORD_STOP = 0x0004,       /* 停止录像 */
    CAMERA_FOCUS_CENTER = 0x0005,      /* 中心对焦 */
    CAMERA_FOCUS_CENTER_STOP = 0x000B  /* 停止中心对焦 */
};

/* 加密类型（帧头 ENC 字段） */
enum RsEncType
{
    RS_ENC_NONE = 0,  /* 不加密（当前实现仅支持该模式） */
    RS_ENC_AES256 = 1 /* AES256 加密（保留） */
};

/* 应答类型（CmdType[4:0]） */
enum RsResponseType
{
    RS_RESP_NONE = 0,     /* 不需要应答 */
    RS_RESP_OPTIONAL = 1, /* 可选应答 */
    RS_RESP_MUST = 2      /* 必须应答 */
};

/* 帧类型（CmdType[5]） */
enum RsFrameType
{
    RS_FRAME_COMMAND = 0, /* 命令帧 */
    RS_FRAME_RESPONSE = 1 /* 应答帧 */
};

/** @brief 参数推送 / 校准状态推送回调（对应协议 0x0E/0x08、0x0E/0x10） */
typedef void (*RsPushCallback)(const RsMessage &msg, void *user);

class RoninS
{
public:
    RoninS();
    explicit RoninS(const rs_can_driver_t &driver);

    /** @brief 初始化协议配置（控制标志、加密类型、CmdType），使用前调用一次 */
    void init();

    /**
     * @brief 绑定/切换 CAN 驱动（会先复制驱动表，再调用它的 init()）
     * @param driver 平台 CAN 驱动接口，send/receive/millis 必须实现
     * @return true = 绑定成功；false = 缺少必要回调，或 driver.init() 失败
     * @note 驱动表按值拷贝，driver 里的 ctx 需要保证生命周期足够长。
     */
    bool attach(const rs_can_driver_t &driver);

    /** @brief 解绑 CAN 驱动（会调用 driver.deinit()，如果实现了的话） */
    void detach();

    /** @brief 是否已绑定 CAN 驱动 */
    bool isAttached() const { return m_driver.send != nullptr; }

    /** @brief 设置等待应答的超时时间（默认 2000ms） */
    void setResponseTimeout(uint32_t ms) { m_timeoutMs = ms; }

    /** @brief 注册参数推送 / 校准状态推送回调 */
    void setPushCallback(RsPushCallback cb, void *user = nullptr);

    /**
     * @brief 处理一次接收（组帧 + 分发），用于空闲时接收推送帧
     * @param timeout_ms 最多等待时间（ms），0 表示非阻塞
     * @return true = 收到了数据（不一定组成完整帧）
     */
    bool poll(uint32_t timeout_ms = 0);

    /** @brief 连接云台：查询 0x00000001 模块版本号确认链路 */
    bool connect();

    /** @brief 断开云台：发送"禁用参数推送"命令 */
    bool disconnect();

    /** @brief 云台设置类接口 */
    class Set
    {
    public:
        explicit Set(RoninS &owner) : m_owner(owner) {}

        /** @brief 设置位置控制中各轴是否有效 */
        bool axis_valid(enum AxisType axis, bool valid);

        /** @brief 设置位置控制模式（绝对 / 增量） */
        bool moveJoint_mode(enum MoveMode type);

        /** @brief 设置速度控制杆与移动速度是否考虑镜头焦距 */
        bool moveSpeed_mode(enum SpeedControl speed_type, enum FocalControl focal_mode);

        /** @brief 2.3.4.4 设置限位角度（各轴 0 ~ 179°） */
        uint8_t limit_angle(uint8_t yaw_max, uint8_t yaw_min,
                            uint8_t pitch_max, uint8_t pitch_min,
                            uint8_t roll_max, uint8_t roll_min);

        /** @brief 2.3.4.6 设置电机力度（0 ~ 100） */
        uint8_t motor_stiffness(uint8_t yaw, uint8_t pitch, uint8_t roll);

        /** @brief 2.3.4.8 参数推送设置：0x00 无操作 / 0x01 使能 / 0x02 禁用 */
        uint8_t param_push(uint8_t ctrl);

        /** @brief 2.3.4.14 设置工作模式与云台方向 */
        uint8_t work_mode(uint8_t work_mode, uint8_t orientation);

        /** @brief 2.3.4.15 云台回中 / 自拍 */
        uint8_t back_center(uint8_t action);

        /** @brief 2.3.4.15 设置云台跟随模式 */
        uint8_t follow_mode(uint8_t follow_mode);

        /** @brief 2.3.4.16 设置自动校准（无应答） */
        uint8_t auto_calibration(uint8_t enable, uint8_t type);

        /** @brief 2.3.4.18 智能跟随（无应答，当前仅支持 cmd = 0x03） */
        uint8_t smart_follow(uint8_t cmd);

        /** @brief 2.3.4.13 设置用户参数（TLV，长度最大 32） */
        uint8_t user_param(uint8_t tlv_id, uint8_t tlv_length, const uint8_t *tlv_data);

    private:
        RoninS &m_owner;
    };

    /** @brief 云台运动类接口 */
    class Move
    {
    public:
        explicit Move(RoninS &owner) : m_owner(owner) {}

        /**
         * @brief 2.3.4.1 位置控制（发送并等待应答）
         * @param yaw/pitch/roll 目标角度（°），范围 -180.0 ~ 180.0
         * @param time_s         运动时间（s），范围 0.1 ~ 25.5
         * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
         */
        uint8_t Joint(float yaw, float pitch, float roll, float time_s);

        /**
         * @brief 2.3.4.1 位置控制（只发送，不等待应答），适合 50ms 周期控制
         * @param seq 可选，回填本帧序列号（2 字节）
         * @return true = 发送成功
         */
        bool JointNoWait(float yaw, float pitch, float roll, float time_s, uint8_t *seq = nullptr);

        /**
         * @brief 2.3.4.2 速度控制（发送并等待应答）
         * @param yaw/pitch/roll 各轴速度（0.1°/s），范围 0 ~ 3600
         * @return true = 云台返回成功
         */
        bool Speed(int16_t yaw, int16_t pitch, int16_t roll);

        /** @brief 2.3.4.11 摇杆控制（无应答），速度单位 0.1°/s */
        void Joystick(int16_t yaw, int16_t pitch, int16_t roll);

        /** @brief 2.3.4.11 拨轮控制（无应答），速度单位 0.1°/s */
        void Dial(int16_t dial_speed);

    private:
        RoninS &m_owner;
    };

    /** @brief 云台查询类接口 */
    class Get
    {
    public:
        explicit Get(RoninS &owner) : m_owner(owner) {}

        /**
         * @brief 2.3.4.3 获取云台角度（单位 0.1°）
         * @param DataType GIMBAL_ATTITUDE_ANGLE / GIMBAL_JOINT_ANGLE
         * @param yaw/pitch/roll 输出角度，可传 nullptr 跳过
         * @return 云台返回码，或 RS_TIMEOUT/RS_ERROR
         */
        uint8_t gimbal_angle(uint8_t DataType, int16_t *yaw, int16_t *pitch, int16_t *roll);

        /** @brief 2.3.4.5 获取限位角度（单位 1°），输出指针可传 nullptr 跳过 */
        uint8_t limit_angle(uint8_t *yaw_max, uint8_t *yaw_min,
                            uint8_t *pitch_max, uint8_t *pitch_min,
                            uint8_t *roll_max, uint8_t *roll_min);

        /** @brief 2.3.4.7 获取电机力度（0 ~ 100），输出指针可传 nullptr 跳过 */
        uint8_t motor_stiffness(uint8_t *yaw, uint8_t *pitch, uint8_t *roll);

        /** @brief 2.3.4.10 获取模块版本号（0xAABBCCDD 表示 AA.BB.CC.DD） */
        uint8_t module_version(uint32_t device_id, uint32_t *version);

        /** @brief 2.3.4.12 获取用户参数（TLV） */
        uint8_t user_param(uint8_t *read_ids, uint8_t id_num, uint8_t *tlv_buffer, uint16_t *tlv_length);

    private:
        RoninS &m_owner;
    };

    /** @brief 跟焦器电机接口（2.3.4.19） */
    class FocusMotor
    {
    public:
        explicit FocusMotor(RoninS &owner) : m_owner(owner) {}

        /** @brief 位置控制（无应答） */
        void move_position(int16_t position);

        /** @brief 电机校准（发送并等待应答），返回云台返回码 */
        uint8_t calibrate(uint8_t MotorType, uint8_t cmd);

        /** @brief 获取电机位置（发送并等待应答），返回云台返回码 */
        uint8_t get_position(uint8_t type, uint8_t *calib_state, uint32_t *position);

    private:
        RoninS &m_owner;
    };

    /** @brief 相机控制接口（2.3.5） */
    class Camera
    {
    public:
        explicit Camera(RoninS &owner) : m_owner(owner) {}

        /** @brief 2.3.5.1 相机动作控制（见 enum CameraAction） */
        uint8_t action(uint16_t action);

        /** @brief 2.3.5 获取相机状态 */
        uint8_t get_status(uint8_t *status);

    private:
        RoninS &m_owner;
    };

    Set set;
    Move move;
    Get get;
    FocusMotor focusMotor;
    Camera camera;

private:
    /* 内部工具：发送 / 等待应答（实现在 Response.cpp） */
    bool sendCmd(uint8_t cmd_set, uint8_t cmd_id, const uint8_t *data,
                 uint16_t data_len, uint8_t *seq_out);
    uint8_t waitResponse(const uint8_t *seq, uint8_t cmd_set, uint8_t cmd_id,
                         uint8_t *out, uint8_t out_max, uint16_t *out_len,
                         uint32_t timeout_ms);
    uint8_t sendAndWait(uint8_t cmd_set, uint8_t cmd_id, const uint8_t *data,
                        uint16_t data_len, uint8_t *out, uint8_t out_max,
                        uint16_t *out_len);

    /* 内部工具：接收组帧与消息分发 */
    bool serviceReceive(uint32_t timeout_ms);
    bool dispatchMessage(const RsMessage &msg);

    /* 待匹配的应答缓冲（固定长度环形队列，不申请内存） */
    static const uint8_t RS_PENDING_MAX = 4;
    void pushPending(const RsMessage &msg);
    bool takePending(const uint8_t *seq, uint8_t cmd_set, uint8_t cmd_id,
                     uint8_t *out, uint8_t out_max, uint16_t *out_len,
                     uint8_t *code);

    rs_can_driver_t m_driver;  /* 平台 CAN 驱动接口（中间层） */
    uint8_t m_positionCtrlByte; /* 位置控制标志字节 */
    uint8_t m_speedCtrlByte;    /* 速度控制标志字节 */
    uint8_t m_enc;              /* 帧头 ENC 字节 */
    uint8_t m_cmdType;          /* 帧头 CmdType 字节 */
    uint32_t m_timeoutMs;       /* 等待应答超时（ms） */
    Combine m_combine;          /* 组帧器 */
    CmdParse m_parser;          /* 帧解析器 */
    uint8_t m_txBuffer[RS_MAX_FRAME_LEN]; /* 发送帧缓冲 */
    RsMessage m_pending[RS_PENDING_MAX];  /* 已收到但还没被匹配的应答 */
    uint8_t m_pendingCount;
    RsPushCallback m_pushCb; /* 推送回调 */
    void *m_pushUser;        /* 推送回调用户参数 */
};

#endif /* DJI_RS_SDK_H */
