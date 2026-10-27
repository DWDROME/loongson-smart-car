#ifndef INIT_H
#define INIT_H

#include "upper_host_config.h"

// 保留旧名称作为主程序接口，真实值只在 upper_host_config.h 定义。
#define SERVER_IP UPPER_HOST_IP
#define PORT UPPER_HOST_PORT
typedef struct 
{
int16_t count_now;
int32_t total_count;       //总脉冲
float speed;         // 名义 count/5ms（滤波后）；非“脉冲/秒”
float rpm;            // 转速（转/秒）
float last_speed;
float D_speed;
}encoder;
extern encoder encoder_L;
extern encoder encoder_R;
// 中环编码器轮差融合权重：[0,1]，0=纯 G_dis，1=纯 Tpm_Dis。
extern float LPF_Speed;
extern zf_device_imu imu_dev;
extern zf_device_ips200 ips200;
extern zf_driver_gpio  key_1;
extern zf_driver_gpio  key_2;
extern zf_driver_gpio  key_3;
extern zf_driver_gpio  key_4;
extern zf_driver_gpio  key_5;// 右按键中键
extern zf_driver_gpio  key_6;//左按键中键
extern zf_driver_adc   key_7;//右按键 左键
extern zf_driver_adc   key_8;//右按键 右键
extern zf_driver_gpio  beep;
extern zf_device_dl1x dl1x_dev;       
extern int8_t run_flag;
extern uint32_t fPS;
extern int32_t it_time, it_time,encode_l_total,encode_r_total;
extern zf_driver_pit pit_timer;
extern int16_t encoder_abs;
extern volatile int16 dl1x_distance_raw ;      // 定时采集的距离原始数据（volatile防止编译器优化）

void init(void);
void Encoder_update(void);
// mode1 的 1m cpm 标定：清零累计脉冲并清空硬件计数器
void Cpm1m_ResetTotals(void);
void LPF_1(float hz,float time,float in,float *out);
void set_pwm(int16_t pwm_l,int16_t pwm_r);
void pid_init(void);

#endif
