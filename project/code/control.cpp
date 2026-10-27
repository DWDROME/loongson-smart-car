#include "zf_common_headfile.hpp"
#include <math.h>
#include <stdio.h>
int16_t speed_now_left,speed_now_right,speed_now,speed_last_left,speed_last_right,pos_goal,pos_now_left,pos_now_right,PWM_L,PWM_R,expect_angle;

// 单位默认值：cpm=35000 → Δcm = 100*avg_count/35000 = avg/350；1m 实测后写 cpm_l/r
float g_count_per_meter = 35000.0f;
float g_count_per_meter_l = 35000.0f;
float g_count_per_meter_r = 35000.0f;
float g_track_width_m = 0.15f;
float g_c_gyro = 28.144800533f; // 拟合闭合后由参数 c_gyro 覆盖
float g_vehicle_yaw_rate = 0.0f;
float g_odometry_count = 0.0f;
float g_gdis_bias = 0.0f;
float g_cgyro_fit = 0.0f;
uint32_t g_cgyro_fit_n = 0;
static float s_cfit_sum_tpm_yaw = 0.0f;
static float s_cfit_sum_yaw2 = 0.0f;

void Middle_GyroStillRecalibrate(void)
{
    // 全模式统一零偏合同（非仅 mode3）：
    // 1) 重采 raw gyroOffset  2) 估 G_dis 残余均值并写入 g_gdis_bias
    // 无死区；启动时车须静置，TRACK/mode1/mode3 同一路径。
    set_pwm(0, 0);
    printf("[G_dis] KEEP STILL: unified gyroOffset + residual bias...\r\n");
    gyroOffset_init();

    const int n = 100;
    float sum = 0.0f;
    float sum_sq = 0.0f;
    for (int i = 0; i < n; ++i)
    {
        ICM_getEulerianAngles();
        const float g = Units_ComputeGdis(icm_data.gyro_y);
        sum += g;
        sum_sq += g * g;
        system_delay_ms(5);
    }
    g_gdis_bias = sum / (float)n;
    const float var = sum_sq / (float)n - g_gdis_bias * g_gdis_bias;
    const float std = (var > 0.0f) ? sqrtf(var) : 0.0f;
    Units_CgyroFitReset();
    printf("[G_dis] residual bias=%.4f std=%.4f c_gyro=%.4f cpmL/R=%.0f/%.0f "
           "(all modes subtract bias). Release hands after this line.\r\n",
           g_gdis_bias, std, g_c_gyro, g_count_per_meter_l, g_count_per_meter_r);
}

float Units_C_model(void)
{
    const float cpm = 0.5f * (g_count_per_meter_l + g_count_per_meter_r);
    return cpm * g_track_width_m * (CONTROL_NOMINAL_DT_MS * 0.001f);
}

float Units_ComputeGdis(float gyro_y_rad_s)
{
    // 唯一符号落点：vehicle_yaw_rate 左转+ = -gyro_y；G_dis = -C_gyro * yaw_rate
    g_vehicle_yaw_rate = -gyro_y_rad_s;
    return -g_c_gyro * g_vehicle_yaw_rate;
}

float Units_NormalizeTpm(float count_l_5ms, float count_r_5ms)
{
    // 把左右轮映射到“平均 cpm”下的等效 count，再做差 → 与 G_dis 同量纲
    const float cpm_l = (g_count_per_meter_l > 1.0f) ? g_count_per_meter_l : g_count_per_meter;
    const float cpm_r = (g_count_per_meter_r > 1.0f) ? g_count_per_meter_r : g_count_per_meter;
    const float cpm_m = 0.5f * (cpm_l + cpm_r);
    if (!(cpm_m > 1.0f))
        return count_l_5ms - count_r_5ms;
    const float cl_n = count_l_5ms * (cpm_m / cpm_l);
    const float cr_n = count_r_5ms * (cpm_m / cpm_r);
    return cl_n - cr_n;
}

void Units_AccumulateOdometry(float avg_count_this_tick)
{
    g_odometry_count += avg_count_this_tick;
}

float Units_CountToCm(float count)
{
    if (!(g_count_per_meter > 1.0f))
        return 0.0f;
    return 100.0f * count / g_count_per_meter;
}

float Units_WheelDeltaToCm(float count_l, float count_r)
{
    const float cpm_l = (g_count_per_meter_l > 1.0f) ? g_count_per_meter_l : g_count_per_meter;
    const float cpm_r = (g_count_per_meter_r > 1.0f) ? g_count_per_meter_r : g_count_per_meter;
    if (!(cpm_l > 1.0f) || !(cpm_r > 1.0f))
        return 0.0f;
    return 50.0f * (count_l / cpm_l + count_r / cpm_r);
}

void Units_CgyroFitReset(void)
{
    s_cfit_sum_tpm_yaw = 0.0f;
    s_cfit_sum_yaw2 = 0.0f;
    g_cgyro_fit_n = 0;
    g_cgyro_fit = 0.0f;
}

void Units_CgyroFitUpdate(float tpm_dis, float yaw_rate_rad_s)
{
    // 仅在有明显转向时累计，避免静止噪声污染
    const float wr = yaw_rate_rad_s;
    if (fabsf(wr) < 0.25f)
        return;
    if (!isfinite((double)tpm_dis) || !isfinite((double)wr))
        return;
    s_cfit_sum_tpm_yaw += tpm_dis * wr;
    s_cfit_sum_yaw2 += wr * wr;
    g_cgyro_fit_n++;
    // 样本过少时噪声极大，不对外给出 C_fit
    if (g_cgyro_fit_n >= 100u && s_cfit_sum_yaw2 > 0.5f)
        g_cgyro_fit = -s_cfit_sum_tpm_yaw / s_cfit_sum_yaw2;
    else
        g_cgyro_fit = 0.0f;
}

float Units_CgyroFitValue(void)
{
    return g_cgyro_fit;
}
PID Velocity = {0};//速度环
PID Velocity_L = {0};//速度环
PID Velocity_R = {0};//速度环
PID picture_distance = {0};//差速转向环
Direction_PID Image = {0};//图像转向环
Direction_PID Dis_1 = {0};//图像转向环
Direction_PID speed_difference = {0};//图像转向环
float V_out,Image_out,Dis_Out,V_out1;//distance1,distance
float Now_Speed,Master_Speed, Kal_Now_Speed, Dis_Speed, Tpm_Dis, G_dis ,last_G_dis,v_left_target,v_right_target,speed_goal,speed_goal1;//当前速度

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut,float K,float q,float f)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->Max_integral = maxI;//输入上限
    pid->Max_output = maxOut;//输出上限
    pid->K =K;
    pid->kq = q;
    pid->kf = f;
    
    
}
/*位置式速度环PID初始化*/ 
void Speed_PID_Init(PID *pid, float kf,float kp, float ki, float kd, float maxI, float maxOut, float max_error)
{   
    pid->kf = kf;//前馈控制系数
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->Max_integral = maxI;//输入上限
    pid->Max_output = maxOut;//输出上限
    pid->Max_error = max_error;
}

// 原因：FF+PI 的 Integral 会跨试次保留；停车必须清零，否则发车瞬间带着旧制动力。
void Speed_PID_Reset(PID *pid)
{
    pid->Integral = 0;
    pid->output = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->Last_feedback = 0;
    pid->Differential = 0;
    pid->ff_term = 0;
    pid->p_term = 0;
    pid->i_term = 0;
    pid->unsaturated = 0;
    pid->sat_flags = 0;
}

// ---- 中环参考（slew 默认 0=直通；正式不挡参考）----
float g_middle_ref_raw = 0.0f;
float g_middle_reference = 0.0f;
float g_middle_ref_slew_per_s = 0.0f;
// 增量式本拍分量（日志）；非 0.5*r 前馈方案
float g_middle_delta = 0.0f;
float g_middle_delta_p = 0.0f;
float g_middle_delta_i = 0.0f;

float Middle_ReferenceUpdate(float raw_ref, float dt_ms)
{
    g_middle_ref_raw = raw_ref;

    if (dt_ms <= 0.0f)
        dt_ms = CONTROL_NOMINAL_DT_MS;

    // slew<=0：直通（正式合同）
    const float slew = g_middle_ref_slew_per_s;
    if (slew <= 0.0f)
    {
        g_middle_reference = g_middle_ref_raw;
        return g_middle_reference;
    }

    const float max_delta = slew * (dt_ms / 1000.0f);
    float delta = g_middle_ref_raw - g_middle_reference;
    delta = LIMIT(delta, -max_delta, max_delta);
    g_middle_reference += delta;
    return g_middle_reference;
}

void Middle_ReferenceReset(void)
{
    g_middle_ref_raw = 0.0f;
    g_middle_reference = 0.0f;
}

void Control_State_Reset(void)
{
    Speed_PID_Reset(&Velocity);
    Speed_PID_Reset(&Velocity_L);
    Speed_PID_Reset(&Velocity_R);
    Dis_1.OutPut = 0;
    Dis_1.Error = 0;
    Dis_1.Last_Error = 0;
    Dis_1.Last_Last_Error = 0;
    Dis_1.Integral = 0;
    Image.OutPut = 0;
    Image.Error = 0;
    Image.Last_Error = 0;
    Image_out = 0;
    Dis_Out = 0;
    v_left_target = 0;
    v_right_target = 0;
    speed_goal = 0;
    PWM_L = 0;
    PWM_R = 0;
    g_middle_delta = 0.0f;
    g_middle_delta_p = 0.0f;
    g_middle_delta_i = 0.0f;
    Middle_ReferenceReset();
}

/*******************************************位置式PID*********************************************/
float Pos_Cal(PID*pid_t,float expect,float feedback){
    pid_t->error = expect - feedback;
    
    pid_t->Integral += pid_t->error;

    if(pid_t->Integral > pid_t->Max_integral)
    pid_t->Integral = pid_t->Max_integral;
    if(pid_t->Integral < -pid_t->Max_integral)
    pid_t->Integral = -pid_t->Max_integral;

    pid_t->output = pid_t->kp * pid_t->error + 
    pid_t->ki *pid_t->Integral + 
    pid_t->kd * (pid_t->error - pid_t->last_error);


    if(pid_t->output > pid_t->Max_output)
    pid_t->output = pid_t->Max_output;
    if(pid_t->output < -pid_t->Max_output)
    pid_t->output = -pid_t->Max_output;

    pid_t ->last_error = pid_t->error;

    return pid_t->output;
    
}
/****************************************************增量式PID**********************************************/
float Inc_Cal(PID *pid_t, float expect, float feedback) {
    pid_t->error = expect - feedback;
     //Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k) + Kd * [e(k) - 2*e(k-1) + e(k-2)]

     float quad_term = 0.0f;

    if (fabsf(feedback) > 50.0f)
    {
        quad_term = pid_t->kq * pid_t->error * fabsf(pid_t->error);
    }

    float delta_output = pid_t->kp * (pid_t->error - pid_t->last_error) +
                         pid_t->ki * pid_t->error +
                         pid_t->kd * (pid_t->error - 2 * pid_t->last_error + pid_t->last_last_error) + quad_term;

    pid_t->output += delta_output;

    float feedforward = pid_t->kf * expect;

    pid_t->final_output = pid_t->output + feedforward;


    if (pid_t->final_output > pid_t->Max_output)
        pid_t->final_output = pid_t->Max_output;
    else if (pid_t->final_output < -pid_t->Max_output)
        pid_t->final_output = -pid_t->Max_output;
    

    pid_t->last_last_error = pid_t->last_error;
    pid_t->last_error = pid_t->error;    

    return pid_t->final_output;
}
/*******************************************位置式速度环 PID：FF+PI 纯函数合同 *********************************************/
// 审查：本拍候选积分抗饱和；全程 float；保存本拍 ff/p/i/unsaturated/sat_flags。
// kd 保留在结构中但不参与计算（旧公式已废弃）。
float Speed_PID_Cal(PID *PID, float Target, float feedback)
{
    PID->feedback = feedback;
    PID->error = Target - feedback;

    const float err_lim = LIMIT(PID->error, -PID->Max_error, PID->Max_error);
    float candidate_I = PID->Integral + err_lim;
    candidate_I = LIMIT(candidate_I, -PID->Max_integral, PID->Max_integral);

    const float ff = PID->kf * Target;
    const float p = PID->kp * PID->error;
    float i_term = PID->ki * candidate_I;
    float candidate_u = ff + p + i_term;

    PID->sat_flags = 0;
    // 本拍同向饱和则拒绝本拍积分增量
    if (candidate_u > PID->Max_output && PID->error > 0.0f)
    {
        PID->sat_flags |= 0x04;
        i_term = PID->ki * PID->Integral;
        candidate_u = ff + p + i_term;
    }
    else if (candidate_u < -PID->Max_output && PID->error < 0.0f)
    {
        PID->sat_flags |= 0x04;
        i_term = PID->ki * PID->Integral;
        candidate_u = ff + p + i_term;
    }
    else
    {
        PID->Integral = candidate_I;
    }

    PID->ff_term = ff;
    PID->p_term = p;
    PID->i_term = i_term;
    PID->unsaturated = candidate_u;
    PID->output = LIMIT(candidate_u, -PID->Max_output, PID->Max_output);
    if (PID->output >= PID->Max_output - 1e-3f)
        PID->sat_flags |= 0x01;
    if (PID->output <= -PID->Max_output + 1e-3f)
        PID->sat_flags |= 0x02;

    PID->Last_feedback = feedback;
    PID->last_error = PID->error;
    return PID->output;
}

/*******************************************图像专用PID*************************************************/
float Image_PID_Calculate(Direction_PID *pid, float expect, float feedback)
{
    pid->Error = expect;

    pid->OutPut = pid->Kp*pid->Error + pid->Kp2*pid->Error*ABS(pid->Error) + pid->Kd*(pid->Error-pid->Last_Error)+ pid->Kd_feedback*feedback;
    pid->OutPut = LIMIT(pid->OutPut,-pid->MAX_OutPut,pid->MAX_OutPut);

    pid->Last_Error = pid->Error;

    return pid->OutPut;
}
/*******************************************中环：作者增量式 PI（ae3379a）+ 执行安全限幅 *****/
// ae3379a 主体：
//   e = r - Dis_Speed
//   delta = Kp*(e - e_last) + Ki*e + Kd*(D_speed_L - D_speed_R)  // 作者 Kd=0
//   Dout += delta
// 作者 LIMIT 被注释 → 现保留 ±MAX_OutPut 作执行器安全边界 + 抗饱和（不照搬无限输出）。
// feedback=Dis_Speed：init 中 LPF*Tpm+(1-LPF)*G；Kd 默认 0。
// 非 0.5*r 前馈；非 middle_ref_slew 挡参考。
float Dis_PID_Calculate(Direction_PID *pid, float expect, float feedback)
{
    // Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + [Kd 轮加速度差，默认 0]
    pid->Error = expect - feedback;

    const float e = pid->Error;
    const float e_last = pid->Last_Error;

    // 作者：Integral 槽位临时存放 Kd*(DL-DR)；Kd=0 时为 0。不用作位置积分。
    const float wheel_d_term = pid->Kd * (encoder_L.D_speed - encoder_R.D_speed);
    pid->Integral = wheel_d_term; // 兼容作者字段用法；勿当 I 状态读

    float delta_p = pid->Kp * (e - e_last);
    float delta_i = pid->Ki * e;

    // 抗饱和：已顶满且误差继续同向推 → 冻结 Ki 增量（保留 Kp 差分以便回落）
    const float u0 = pid->OutPut;
    const float max_u = pid->MAX_OutPut;
    if ((u0 >= max_u - 1e-3f && e > 0.0f) || (u0 <= -max_u + 1e-3f && e < 0.0f))
        delta_i = 0.0f;

    const float delta = delta_p + delta_i + wheel_d_term;
    g_middle_delta_p = delta_p;
    g_middle_delta_i = delta_i;
    g_middle_delta = delta;

    pid->OutPut += delta;
    // 相对作者：开启输出限幅（必要安全修正）
    pid->OutPut = LIMIT(pid->OutPut, -max_u, max_u);

    pid->Last_Last_Error = pid->Last_Error;
    pid->Last_Error = e;
    pid->expect_last = expect;
    return pid->OutPut;
}


float speed_difference_Calculate(Direction_PID *pid, float expect, float feedback)
{
    pid->Error = expect-feedback;

    pid->OutPut = pid->Kp*pid->Error + pid->Kp2*pid->Error*ABS(pid->Error) + pid->Kd*(pid->Error-pid->Last_Error)+ pid->Kd_feedback*feedback;

    pid->Last_Error = pid->Error;

    return pid->OutPut;
}
/*******************************************方向控制*************************************************/
float Image_E1, Image_E2 = 0;
// void Dir_Control(){

//     Image_PID_Calculate(&Image, Image_E2 , icm_data.gyro_z);//方向环（外环）
//     Inc_Cal(&Dis, Image.OutPut , Dis_Speed);//方向环（内环）

//     // PWM_L = Velocity_L.output + Dis.output;
//     // PWM_R = Velocity_R.output + Dis.output;

//     set_pwm(PWM_L,PWM_R);
// }

    void Direction_PID_Init()   
    {
    // //forward = 28;//打角行



    float K = 2.5;//2.3//12
    Image.Kp = 3.5*K;//  0.8; //250*0.015
    Image.Ki = 0;   
    Image.Kd =2;// Image.Kp *0.3
    Image.Kd_feedback =Image.Kp * 0;
    Image.Kp2 = 0;
    Image.MAX_OutPut = 10000;

    // 中环 ae3379a 增量 PI：Kp=0.7 Ki=0.3（5*0.06）Kd=0；max 由参数（安全 70）
    Dis_1.Ki = 0.30f; // 5*0.06
    Dis_1.Kp = 0.70f;
    Dis_1.Kd = 0;
    Dis_1.MAX_OutPut = 70;
    Dis_1.OutPut = 0;
    Dis_1.Error = 0;
    Dis_1.Last_Error = 0;
    Dis_1.Integral = 0;
    g_middle_ref_slew_per_s = 0.0f;


    float K_v = 1.0f;
    // Max_output = MOTOR_PWM_LIMIT；默认 Kf 用实车草稿 6.2，单位闭合后仍须重标
    Speed_PID_Init(&Velocity, 6.2f * K_v, 30 * K_v, 0, 0, 1500, MOTOR_PWM_LIMIT, 2000);
    Speed_PID_Init(&Velocity_L, 6.2f * K_v, 30 * K_v, 0, 0, 1500, MOTOR_PWM_LIMIT, 2000);
    Speed_PID_Init(&Velocity_R, 6.2f * K_v, 30 * K_v, 0, 0, 1500, MOTOR_PWM_LIMIT, 2000);

    // float K_v=2.0;
    // Speed_PID_Init(&Velocity,5*K_v,1*K_v,0,0*K_v,1500,10000,2000); //位置式速度环.
    // Speed_PID_Init(&Velocity_L,5*K_v,1*K_v,0,0*K_v,1500,10000,2000); //位置式速度环.
    // Speed_PID_Init(&Velocity_R,5*K_v,1*K_v,0,0*K_v,1500,10000,2000); //位置式速度环.

    PID_Init(&picture_distance, -10, 0, -5, 0, 1000,0,0,0);



    // speed_difference.Kp=1.0;
    // speed_difference.Kd=speed_difference.Kp*0.5;
    // float V_i=2;
    // Speed_PID_Init(&Velocity,0,V_i*10,V_i,V_i*0.75 ,10000,10000,2000);
    // Speed_PID_Init(&Velocity_L,0,V_i*10,V_i,V_i*1.5,10000,10000,2000);
    // Speed_PID_Init(&Velocity_R,0,V_i*10,V_i,V_i*1.5,10000,10000,2000);
    // Speed_PID_Init(&Velocity_L,2,35,0.5,18,3000,8000,100);
    // Speed_PID_Init(&Velocity_R,2,35,0.5,18,3000,8000,100);
    
}
