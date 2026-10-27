#ifndef RUN_MODE_H
#define RUN_MODE_H

#include <stdint.h>

// 原因：运行模式合同单一枚举；值与 remote/key_supervisor 一致，禁止未知值静默当运行。
enum RunMode : int8_t
{
    RUN_MODE_STOP = 0,         // 停车，PWM=0，清状态
    RUN_MODE_SPEED_TEST = 1,   // 左右速度环：target=base（base<=0 时 target=0 仍闭环）
    RUN_MODE_SENSOR_CAL = 2,   // 锁电机，只采传感器
    RUN_MODE_MIDDLE_TEST = 3,  // 速度 + 中环，视觉旁路
    RUN_MODE_TRACK = 10,       // 完整三环巡线
    RUN_MODE_IMAGE_TEST = 30,  // 锁电机图像/调试
    RUN_MODE_CAMERA_FOCUS = 31,// 锁电机原生 320x240 镜头清晰度调试
};

// 输出互斥：比赛默认 none；VOFA 只显式调试；assistant 后续接逐飞。
enum OutputMode : int8_t
{
    OUTPUT_MODE_NONE = 0,
    OUTPUT_MODE_VOFA = 1,
    OUTPUT_MODE_ASSISTANT = 2,
};

inline bool IsKnownRunMode(int value)
{
    return value == RUN_MODE_STOP ||
           value == RUN_MODE_SPEED_TEST ||
           value == RUN_MODE_SENSOR_CAL ||
           value == RUN_MODE_MIDDLE_TEST ||
           value == RUN_MODE_TRACK ||
           value == RUN_MODE_IMAGE_TEST ||
           value == RUN_MODE_CAMERA_FOCUS;
}

// 电机应关闭的模式
inline bool RunModeMotorsLocked(int mode)
{
    return mode == RUN_MODE_STOP ||
           mode == RUN_MODE_SENSOR_CAL ||
           mode == RUN_MODE_IMAGE_TEST ||
           mode == RUN_MODE_CAMERA_FOCUS;
}

extern int8_t g_run_mode;
extern int8_t g_output_mode;
// SPEED_TEST 专用：1=手推 cpm（PWM0、清状态、不跑速度环）；0=默认闭环（base<=0 为零速 PI）
// 仅 --freewheel 或 SPEED_FREEWHEEL=1 置位；不得用 base_speed=0 冒充松电机
extern int8_t g_speed_freewheel;
extern float g_base_speed;          // 直道/默认巡线目标
extern float g_straight_trim;       // 直线偏置；>0 抬左压右，TRACK 叠加后仍受中环限幅
extern float g_picture_speed;       // picture==2 阶段速度预设
extern float g_ramp_speed;          // ramp==2 阶段速度预设
extern float g_picture_recog_k;     // 开扫距离 = Now_Speed*k + base
extern float g_picture_recog_base;
extern float g_picture_near_dist;   // 近距再检距离门限（legacy distance）
extern float g_picture_side_offset; // 根提交 picture 3/4 贴边偏置（像素）
extern float g_picture_hold_dist;   // picture 3/4/5 结束里程
extern float g_picture_finish_dist; // picture 6 收尾里程
extern float g_picture_conf_min;    // 三分类置信度门槛，默认 0.50
extern float g_middle_test_target;  // 中环人工目标（模式3）
extern float g_image_kp_scale;      // 外环 Kp 比例（乘 Direction 初值）
extern uint32_t g_control_tick;
extern float g_control_dt_ms;

#endif
