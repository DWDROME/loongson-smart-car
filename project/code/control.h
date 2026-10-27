 #ifndef CONTROL_H
 #define CONTROL_H

#define LIMIT(input, low, upper)    MIN(MAX(input, low), upper)//限幅
#define ABS(a) ((a >= 0) ? (a) : (-a))
// /************************************PID***********************************/

// 电机 PWM 软上限：速度环 Max_output 与 set_pwm 必须同一常数（审查合同）
#ifndef MOTOR_PWM_LIMIT
#define MOTOR_PWM_LIMIT 5000.0f
#endif
// 名义控制拍（ms）：编码器归一化为 count/k_nominal_dt_ms
#ifndef CONTROL_NOMINAL_DT_MS
#define CONTROL_NOMINAL_DT_MS 5.0f
#endif
// TRACK 图像过期（ms）：超过则硬停，不沿用旧 Image_out
#ifndef IMAGE_STALE_MS
#define IMAGE_STALE_MS 200.0f
#endif

typedef struct
{
    float kp, ki, kd;
    float kq;//平方项系数
    float kf;//前馈项系数
    float error,last_error,last_last_error;//误差 上次误差 上上次误差
    float Max_error;
    float feedback,Last_feedback;//反馈值 上次反馈值
    float Differential;//微分项
    float Integral, Max_integral;
    float output, Max_output,final_output;
    // 本拍真分量（审查：VOFA 直接读，不重算）
    float ff_term;
    float p_term;
    float i_term;
    float unsaturated;
    // bit0: 输出正饱和 bit1: 输出负饱和 bit2: 本拍拒绝积分
    unsigned char sat_flags;
    float K;
}PID;

typedef struct{
        float Kp;//一次项系数
        float Kp2;//二次项系数
        float Ki;
        float Kd;
        float Kd_feedback;      //反馈值微分系数

        float Error;            //误差
        float Integral;         //误差积分
        float Max_Error;            //误差
        float Last_Error;       //上次误差
        float Last_Last_Error;

        float OutPut;
        float MAX_Integral;
        float MAX_OutPut;
        float expect_last;

}Direction_PID;
extern Direction_PID Image, Rate,Dis_1,speed_difference;
extern PID Dis,picture_distance;//差速转向环
extern PID Velocity;//速度环
extern PID Velocity_L;//速度环
extern PID Velocity_R;//速度环
extern PID Groy_turn;
extern PID Image_turn;
extern int16_t speed_now_left,speed_now_right,speed_now,speed_last_left,speed_last_right,pos_goal,pos_now_left,pos_now_right,PWM_L,PWM_R,expect_angle;
extern int8_t run_flag;
extern float Image_E1, Image_E2, v_left_target,v_right_target,speed_goal,speed_goal1;
extern float Now_Speed, Kal_Now_Speed, Dis_Speed, Tpm_Dis, G_dis,last_G_dis,Master_Speed;//当前速度
extern float V_out,V_out1,Image_out,Dis_Out,distance2,distance1;
float Pos_Cal(PID*pid_t,float expect,float feedback);
float Inc_Cal(PID *pid_t, float expect, float feedback);
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut,float K,float q,float f);
// target/feedback 全程 float；输出为限幅后的 PWM 量（与 MOTOR_PWM_LIMIT 同量纲）
float Speed_PID_Cal(PID *PID, float Target, float feedback);
void Speed_PID_Init(PID *pid, float kf,float kp, float ki, float kd,float maxI, float maxOut, float max_error);
// 停车/停机时清速度环状态，避免下次发车继承积分制动力（FF+PI R4）
void Speed_PID_Reset(PID *pid);
// 控制拍实际间隔（ms），由 pit 写入；编码器归一化读取
extern float g_control_dt_ms;
extern uint32_t g_control_tick;

// ---- 单位合同：名义控制 = count/5ms；里程 = cm ----
// count_per_meter_l/r：左右轮各自计数/米（1m 实测）；g_count_per_meter = 均值（兼容旧键）
// track_width_m：轮距（米），C_model 量级检查
// c_gyro：G_dis 整体系数（count/5ms）/(rad/s)；由 C_fit 闭合后写入参数
extern float g_count_per_meter;
extern float g_count_per_meter_l;
extern float g_count_per_meter_r;
extern float g_track_width_m;
extern float g_c_gyro;
extern float g_vehicle_yaw_rate; // 左转为正 rad/s
extern float g_odometry_count;   // 累计里程真源（等效平均 raw count）
extern float g_cgyro_fit;        // 在线拟合结果；样本不足时为 0
extern uint32_t g_cgyro_fit_n;   // 有效拟合样本数
float Units_C_model(void);       // mean_cpm * track_width * 0.005

// 全模式共用：静置估得的 G_dis 残余偏置（非死区）；pit 内 G_dis 一律减去
extern float g_gdis_bias;
// 统一零偏：重采 GyroOffset + 估 g_gdis_bias。TRACK/mode1/mode3 启动均应调用
void Middle_GyroStillRecalibrate(void);
// 输入 icm_data.gyro_y（rad/s，减零偏后）；内部写 g_vehicle_yaw_rate，返回未减 residual 的 G_dis
float Units_ComputeGdis(float gyro_y_rad_s);
// 左右轮 count 差 → 尺度归一后的 Tpm（与 G_dis 同量纲 count/5ms）
float Units_NormalizeTpm(float count_l_5ms, float count_r_5ms);
void Units_AccumulateOdometry(float avg_count_this_tick);
// 旧接口：用平均 cpm；新里程请用 Units_WheelDeltaToCm
float Units_CountToCm(float count);
// 左右本拍 raw count → cm（分 cpm）
float Units_WheelDeltaToCm(float count_l, float count_r);
// 在线 C_gyro 拟合：C_fit = -sum(Tpm*yaw_rate)/sum(yaw_rate^2)
void Units_CgyroFitReset(void);
void Units_CgyroFitUpdate(float tpm_dis, float yaw_rate_rad_s);
float Units_CgyroFitValue(void);
// ---- 中环参考（slew 默认 0=直通；正式不挡参考）----
extern float g_middle_ref_raw;
extern float g_middle_reference;
extern float g_middle_ref_slew_per_s; // 参数 middle_ref_slew；0=直通
float Middle_ReferenceUpdate(float raw_ref, float dt_ms);
void Middle_ReferenceReset(void);

// 作者增量式中环本拍分量（日志）
extern float g_middle_delta;   // Δu
extern float g_middle_delta_p; // Kp*(e-e_last)
extern float g_middle_delta_i; // Ki*e（抗饱和后）

// 原因：模式切换时统一清速度/中环/外环状态，不重构控制器。
void Control_State_Reset(void);
void Direction_PID_Init(void);
float Image_PID_Calculate(Direction_PID *pid, float expect, float feedback);
// 作者增量 PI：expect=r=-Image_out，feedback=G_dis；OutPut 有记忆
float Dis_PID_Calculate(Direction_PID *pid, float expect, float feedback);
float speed_difference_Calculate(Direction_PID *pid, float expect, float feedback);




 #endif