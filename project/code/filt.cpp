#include "zf_common_headfile.hpp"
#include "math.h"
int16_t imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z;
int16_t imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z;
// /**************************************************************************
// 函数功能：简易卡尔曼滤波
// 入口参数：加速度、角速度
// 返回  值：无
// 作    者：平衡小车之家
// **************************************************************************/
// float angle = 0, angle_dot = 0;
// float Q_angle=0.001;// 过程噪声的协方差
// float Q_gyro=0.003;//0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
// float R_angle=0.5;// 测量噪声的协方差 既测量偏差
// float dtt=0.001;//    积分时间
// char  C_0 = 1;
// float Q_bias = 0, Angle_err = 0;
// float PCt_0 = 0, PCt_1 = 0, E = 0;
// float K_0 = 0, K_1 = 0, t_0 = 0, t_1 = 0;
// float Pdot[4] ={0,0,0,0};
// float PP[2][2] = { { 1, 0 },{ 0, 1 } };
// void Kalman_Filter(float Accel,float Gyro)
// {
//     angle+=(Gyro - Q_bias) * dtt; //先验估计
//     Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

//     Pdot[1]=-PP[1][1];
//     Pdot[2]=-PP[1][1];
//     Pdot[3]=Q_gyro;
//     PP[0][0] += Pdot[0] * dtt;   // Pk-先验估计误差协方差微分的积分
//     PP[0][1] += Pdot[1] * dtt;   // =先验估计误差协方差
//     PP[1][0] += Pdot[2] * dtt;
//     PP[1][1] += Pdot[3] * dtt;

//     Angle_err = Accel - angle;  //zk-先验估计

//     PCt_0 = C_0 * PP[0][0];
//     PCt_1 = C_0 * PP[1][0];

//     E = R_angle + C_0 * PCt_0;

//     K_0 = PCt_0 / E;
//     K_1 = PCt_1 / E;

//     t_0 = PCt_0;
//     t_1 = C_0 * PP[0][1];

//     PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
//     PP[0][1] -= K_0 * t_1;
//     PP[1][0] -= K_1 * t_0;
//     PP[1][1] -= K_1 * t_1;

//     angle   += K_0 * Angle_err;  //后验估计
//     Q_bias  += K_1 * Angle_err;  //后验估计
//     angle_dot   = Gyro - Q_bias;     //输出值(后验估计)的微分=角速度
// }

// /**************************************************************************
// 函数功能：一阶互补滤波
// 入口参数：加速度、角速度
// 返回  值：无
// 作    者：平衡小车之家
// **************************************************************************/
// float K1=0.02;
// void Yijielvbo(float angle_m, float gyro_m)
// {

//   angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dtt);
// }

// //二阶互补滤波
// /**************************************************************************
// 函数功能：二阶互补滤波
// 入口参数：加速度、角速度
// 返回  值：无
// 作    者：平衡小车之家
// **************************************************************************/
// //float K2 =0.2; // 对加速度计取值的权重
// //float x1,x2,y1;
// //float dtt=20*0.001;//注意：dtt的取值为滤波器采样时间
// //float angle2;
// float K2 =0.15; // 对加速度计取值的权重
// float x1 = 0,x2 = 0,y1_value = 0;
// void Erjielvbo(float angle_m, float gyro_m)//采集后计算的角度和角加速度
// {
//     x1=(angle_m-angle)*(1-K2)*(1-K2);
//     y1_value=y1_value+x1*dtt;
//     x2=y1_value+2*(1-K2)*(angle_m-angle)+gyro_m;
//     angle=angle+ x2*dtt;
// }


// /****************************滑动均值滤波模板********************************
// int Car_nowspeed = 0;  //小车当前速度
// int HD_speed_last[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// int HD_speed_temp = 0;
// int HD_speed_sum = 0;

//     HD_speed_temp = (int)((encoder_left.Pulse + encoder_right.Pulse) / 2);

//     for(int i = 19; i > 0; i--)
//     {
//         HD_speed_last[i] = HD_speed_last[i-1];
//     }
//     HD_speed_last[0] = HD_speed_temp;
//     for(int i = 19; i >= 0; i--)
//     {
//         HD_speed_sum += HD_speed_last[i];
//     }

//     Car_nowspeed = (int)(HD_speed_sum/20);   //速度滤波
//     HD_speed_sum = 0;
// ****************************************************************************/

/**************************************滤波***************************************/

/**
 * @brief 一阶低通滤波器（基于截止频率和采样时间）
 * @param hz 截止频率（单位：Hz），决定滤波器允许通过的最高频率
 * @param time 采样时间间隔（单位：秒），即函数调用周期
 * @param in 当前时刻的输入值（待滤波的原始信号）
 * @param out 指向滤波结果的指针，同时作为输入（上次滤波值）和输出（本次滤波结果）
 * @note 传递函数模型：G(s) = 1 / (τs + 1)，其中 τ = 1/(2π·hz) 为时间常数
 *        离散化公式：y(k) = y(k-1) + α·[x(k) - y(k-1)]，α = T/(T + τ)
 */

void LPF_1(float hz,float time,float in,float *out)
{   
    float alpha = 1 / (1 + 1 / (hz * 6.28f * time));

    *out += alpha * (in - *out);
}

void imu660ra_get_acc(void)
{
        imu660ra_acc_x = imu_dev.get_acc_x();//
        imu660ra_acc_y = imu_dev.get_acc_y();//
        imu660ra_acc_z = -imu_dev.get_acc_z();//

        // imu660ra_acc_x = imu_dev.get_acc_x()*2;//
        // imu660ra_acc_y = imu_dev.get_acc_y()*2;//
        // imu660ra_acc_z = imu_dev.get_acc_z()*2;//
}

void imu660ra_get_gyro(void)
{
        imu660ra_gyro_x = imu_dev.get_gyro_x();//
        imu660ra_gyro_y = imu_dev.get_gyro_y();//
        imu660ra_gyro_z = -imu_dev.get_gyro_z();//

        // imu660ra_gyro_x = imu_dev.get_gyro_x()*2;//
        // imu660ra_gyro_y = imu_dev.get_gyro_y()*2;//
        // imu660ra_gyro_z = imu_dev.get_gyro_z()*2;//
}



/**************************************陀螺仪解算***********************************/
//IMU660RA 数据 加速度计量程：8g    -32768 ~ +32767(65535)   1g->4096
//IMU陀螺仪量程 2000度每秒          -32768 ~ +32768(65535)  65535/4000 = 16.4

#define delta_T     0.005f

#ifndef M_PI
#define M_PI        3.1415926f
#endif
//float icm_ay,icm_sy;



float I_ex, I_ey, I_ez;
float online_bias_x = 0, online_bias_y = 0, online_bias_z = 0;
int bias_update_count = 0;

quater_param_t Q_info = {1, 0, 0};


icm_param_t icm_data;
gyro_param_t GyroOffset;

uint8 GyroOffset_init = 0;

float imu_gyro_z_val=0;
float param_Kp = 0.17f;
float param_Ki = 0.004f;//


float fast_sqrt(float x)
{
    // 处理特殊值：x≤0时直接返回0（原算法无处理，会出错）
    if (x <= 0.0f) return 0.0f;

    float halfx = 0.5f * x;
    float y = x;
    uint32_t i; // 用32位无符号整数，匹配float的32位

    // 安全的位级拷贝（避免严格别名问题）
    memcpy(&i, &y, sizeof(i));

    // 魔法数+整数位运算：近似平方根的整数表示
    i = 0x5f3759df - (i >> 1);

    // 转回float
    memcpy(&y, &i, sizeof(y));

    // 牛顿迭代1次（如需更高精度，可再迭代1次）
    y = y * (1.5f - (halfx * y * y));

    // 注意：原逻辑的y是√x的近似值，但迭代后需修正（最终乘以x才是更准的√x）
    // 修正版：y = y * x; // 若发现结果偏小，添加这行
    return y;
}


void gyroOffset_init(void)
{
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
    for (uint16 i = 0; i < 500; ++i)
    {
        imu660ra_get_acc();                                     // 获取 IMU660RA 加速度计数据
        imu660ra_get_gyro();

        GyroOffset.Xdata += imu660ra_gyro_x;
        GyroOffset.Ydata += imu660ra_gyro_y;
        GyroOffset.Zdata += imu660ra_gyro_z;

        system_delay_ms(1);
    }

    GyroOffset.Xdata /= 500;
    GyroOffset.Ydata /= 500;
    GyroOffset.Zdata /= 500;
    GyroOffset.AXdata /= 500;
    GyroOffset.AYdata /= 500;
    GyroOffset.AZdata /= 500;

    GyroOffset_init = 1;
}

#define alpha         0.3f


void ICM_getValues()
{
    icm_data.acc_x = ((float) imu660ra_acc_x* alpha) * 8 / 4096 + icm_data.acc_x * (1 - alpha);
    icm_data.acc_y = ((float) imu660ra_acc_y * alpha) * 8 / 4096 + icm_data.acc_y * (1 - alpha);
    icm_data.acc_z = ((float) imu660ra_acc_z* alpha) * 8 / 4096 + icm_data.acc_z * (1 - alpha);

//    icm_data.acc_x = ((float) imu660ra_acc_x-GyroOffset.AXdata) * 8 / 4096;
//    icm_data.acc_y = ((float) imu660ra_acc_y-GyroOffset.AYdata) * 8 / 4096;
//    icm_data.acc_z = ((float) imu660ra_acc_z-GyroOffset.AZdata) * 8 / 4096 ;



    icm_data.gyro_x = ((float) imu660ra_gyro_x - GyroOffset.Xdata) * M_PI / 180 / 16.4f;
    icm_data.gyro_y = ((float) imu660ra_gyro_y - GyroOffset.Ydata) * M_PI / 180 / 16.4f;
    icm_data.gyro_z = ((float) imu660ra_gyro_z - GyroOffset.Zdata) * M_PI / 180 / 16.4f;

//    icm_ay=icm_data.acc_y ;
//    icm_sy=icm_data.gyro_y;
}



void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float halfT = 0.5 * delta_T;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
//    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
//    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    // float delta_2 = 0;

    float norm = fast_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    //vz = (q0*q0-0.5f+q3 * q3) * 2;


    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;


    I_ex += halfT * ex;   // integral error scaled by Ki
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    gx = gx + param_Kp * ex + param_Ki * I_ex;
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;



    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    //    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    //    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    //    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    //    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    //    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT


    // normalise quaternion
    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}

void ICM_getEulerianAngles(void)
{
      imu660ra_get_gyro();
      imu660ra_get_acc();                                     // 获取 IMU660RA 加速度计数据


    ICM_getValues();
    ICM_AHRSupdate(icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z, icm_data.acc_x, icm_data.acc_y, icm_data.acc_z);
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;


    icm_data.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / M_PI; // pitch
    icm_data.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI; // roll
    icm_data.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI;//转向角
}


void huandao_yaw_correct(void)
{
    if(Yaw_Huandao>=0&&Yaw_Huandao<=180)
    {
        if((icm_data.yaw>=-180)&&icm_data.yaw<(-180+Yaw_Huandao))
        {
            yaw_correct=icm_data.yaw+360;
            Yaw_Huandao_err=Yaw_Huandao-yaw_correct;
        }
        else Yaw_Huandao_err=Yaw_Huandao-icm_data.yaw;
    }

    else if(Yaw_Huandao<0&&Yaw_Huandao>=-180)
    {
        if(icm_data.yaw<=180&&icm_data.yaw>(180+Yaw_Huandao))
        {
            yaw_correct=icm_data.yaw-360;
            Yaw_Huandao_err=Yaw_Huandao-yaw_correct;
        }
        else Yaw_Huandao_err=Yaw_Huandao-icm_data.yaw;
    }
}



