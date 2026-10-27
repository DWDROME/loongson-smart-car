#include "zf_common_headfile.hpp"
/*存放 定时器 电机 Flash 按键 编码器 蜂鸣器等外设的初始化*/
#define KEY_1_PATH        ZF_GPIO_KEY_1
#define KEY_2_PATH        ZF_GPIO_KEY_2
#define KEY_3_PATH        ZF_GPIO_KEY_3
#define KEY_4_PATH        ZF_GPIO_KEY_4
#define KEY_5_PATH        ZF_GPIO_KEY_5
#define KEY_6_PATH        ZF_GPIO_KEY_6

#define BEEP_PATH        ZF_GPIO_BEEP
// 原因：Car_Round 编码器合同 — 左 QUAD2 前进+，右 QUAD1 读后取负为前进+（见 hardware-direction-matrix）。
#define ENCODER_DIR_1_PATH           ZF_ENCODER_QUAD_2
#define ENCODER_DIR_2_PATH           ZF_ENCODER_QUAD_1
#define PWM_1_PATH        ZF_PWM_MOTOR_1
#define DIR_1_PATH        ZF_GPIO_MOTOR_1
#define PWM_2_PATH        ZF_PWM_MOTOR_2
#define DIR_2_PATH        ZF_GPIO_MOTOR_2
// 原因：实车无 ESC，已从有效路径删除 ZF_PWM_ESC_1（PRD R2.2）。
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX    (drv8701e_pwm_1_info.duty_max)       
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。 
#define MOTOR2_PWM_DUTY_MAX    (drv8701e_pwm_2_info.duty_max) 
zf_driver_gpio  key_1(KEY_1_PATH, O_RDWR);//下
zf_driver_gpio  key_2(KEY_2_PATH, O_RDWR);//右
zf_driver_gpio  key_3(KEY_3_PATH, O_RDWR);//上
zf_driver_gpio  key_4(KEY_4_PATH, O_RDWR);//左
zf_driver_gpio  key_5(KEY_5_PATH, O_RDWR);// 右按键中键
zf_driver_gpio  key_6(KEY_6_PATH, O_RDWR);//左按键中键
zf_driver_adc   key_7(ADC_CH3_PATH);//右按键 左键
zf_driver_adc   key_8(ADC_CH1_PATH);//右按键 右键
zf_driver_gpio  beep(BEEP_PATH, O_RDWR);
zf_driver_encoder encoder_dir_1(ENCODER_DIR_1_PATH);
zf_driver_encoder encoder_dir_2(ENCODER_DIR_2_PATH);
struct pwm_info drv8701e_pwm_1_info;
struct pwm_info drv8701e_pwm_2_info;
zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);
zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);
zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);
zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);
zf_device_imu imu_dev;
zf_device_ips200 ips200;
int32_t encoder_left,encoder_right;
encoder encoder_L = {0};
encoder encoder_R = {0};
// 中环反馈融合：Dis_Speed = LPF*Tpm + (1-LPF)*G。
// ae3379a 提交为 0；由 middle_tpm_weight 显式加载后才启用编码器融合。
float LPF_Speed = 0.0f;
int8_t run_flag;
uint32_t fPS;
int16_t encoder_abs;
int32_t it_time,encode_l_total,encode_r_total;
zf_driver_pit pit_timer;
extern zf_device_uvc uvc_dev;

zf_device_dl1x dl1x_dev;                  // DL1X设备对象
enum dl1x_device_type_enum dl1x_dev_type;  // DL1X设备类型
zf_driver_pit dl1x_pit_timer;             // PIT定时器对象（用于100ms定时采集）
volatile int16 dl1x_distance_raw = 0;      // 定时采集的距离原始数据（volatile防止编译器优化）


void pid_init(void)
{
    run_flag=0;
    //720 duty 小车刚好能在地面上静止
    Direction_PID_Init();//转向环 正为左转 负为右转

    //Speed_PID_Init(&Velocity,3,20,0.1,6,5000,10000,30);//位置式速度环

}
void sigint_handler(int signum) 
{
    drv8701e_pwm_1.set_duty(0);   
    drv8701e_pwm_2.set_duty(0);  
    printf("收到Ctrl+C,程序即将退出\n");
    exit(0);
}

void cleanup()
{
    pit_timer.stop();
    uvc_dev.close();
    // 关闭电机
    drv8701e_pwm_1.set_duty(0);   
    drv8701e_pwm_2.set_duty(0);    
    printf("程序异常退出，执行清理操作\n");
}


void init()
{   atexit(cleanup);//清理函数注册
    signal(SIGINT, sigint_handler);// 注册SIGINT信号的处理函数
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);//电机初始化 获取设备信息
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);//电机初始化 获取设备信息
    imu_dev.init();// IMU传感器初始化，自动检测挂载的IMU型号并完成底层配置
    gyroOffset_init();
    dl1x_dev.init();
    ips200.init(FB_PATH);//屏幕初始化
    encode_l_total = 0;
    encode_r_total = 0;
    encoder_abs = 0;
}

void Cpm1m_ResetTotals(void)
{
    encode_l_total = 0;
    encode_r_total = 0;
    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();
    encoder_L.count_now = 0;
    encoder_R.count_now = 0;
}

void Encoder_update()
{

    //转速 = (脉冲数 / 编码器线数) / 采样时间
    // 原因：去掉软件 ×2，沿用 Car_Round 周期 count 口径；右轮取负使前进为正。
    encoder_L.count_now = encoder_dir_1.get_count();
    encoder_R.count_now = -encoder_dir_2.get_count();
    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();
    // cpm 标定累计：mode1（手推 base=0 或电机自走）
    if (g_run_mode == RUN_MODE_SPEED_TEST)
    {
        encode_l_total += (int32_t)encoder_L.count_now;
        encode_r_total += (int32_t)encoder_R.count_now;
    }


    // 名义单位：count / CONTROL_NOMINAL_DT_MS（默认 5ms）。用实际 g_control_dt_ms 归一化，避免漏拍胀大“速度”。
    float dt_ms = g_control_dt_ms;
    if (dt_ms < 1.0f)
        dt_ms = CONTROL_NOMINAL_DT_MS;
    if (dt_ms > 50.0f)
        dt_ms = 50.0f;
    const float norm = CONTROL_NOMINAL_DT_MS / dt_ms;
    const float count_l_5ms = (float)encoder_L.count_now * norm;
    const float count_r_5ms = (float)encoder_R.count_now * norm;

    LPF_1(20, 5.0e-3, count_l_5ms, &encoder_L.speed);
    LPF_1(20, 5.0e-3, count_r_5ms, &encoder_R.speed);

    LPF_1(20, 5.0e-3, 0.75f * MAX(encoder_L.speed, encoder_R.speed) + 0.25f * MIN(encoder_L.speed, encoder_R.speed), &Master_Speed);
    LPF_1(20, 5.0e-3, 0.5f * (count_l_5ms + count_r_5ms), &Now_Speed);
    // Tpm：左右 cpm 归一后再差；可参与中环反馈融合 + C_gyro 拟合
    {
        const float tpm_raw = Units_NormalizeTpm(encoder_L.speed, encoder_R.speed);
        LPF_1(20, 5.0e-3, tpm_raw, &Tpm_Dis);
    }

        encoder_L.D_speed=encoder_L.speed-encoder_L.last_speed;
        encoder_R.D_speed=encoder_R.speed-encoder_R.last_speed;
        encoder_L.last_speed=encoder_L.speed;
        encoder_R.last_speed=encoder_R.speed;

    // G_dis 合同：Units_ComputeGdis → 减全模式统一 residual bias
    G_dis = Units_ComputeGdis(icm_data.gyro_y) - g_gdis_bias;
    // 回退旧 Z 轴：G_dis = Units_ComputeGdis(-icm_data.gyro_z); 且需重标 g_c_gyro

    // 中环反馈：作者结构 LPF*Tpm+(1-LPF)*G；权重由 middle_tpm_weight 加载。
    // 不能修复错误几何参考；Kd 轮差动态项未标定，保持 0。
    Dis_Speed = LPF_Speed * Tpm_Dis + (1.0f - LPF_Speed) * G_dis;

    // 在线 C_gyro 拟合样本（转向足够时）
    Units_CgyroFitUpdate(Tpm_Dis, g_vehicle_yaw_rate);

    // 里程：左右分 cpm 的等效平均 count 累加
    {
        const float cpm_l = (g_count_per_meter_l > 1.0f) ? g_count_per_meter_l : g_count_per_meter;
        const float cpm_r = (g_count_per_meter_r > 1.0f) ? g_count_per_meter_r : g_count_per_meter;
        const float cpm_m = 0.5f * (cpm_l + cpm_r);
        const float eq_avg =
            0.5f * ((float)encoder_L.count_now * (cpm_m / cpm_l) +
                    (float)encoder_R.count_now * (cpm_m / cpm_r));
        Units_AccumulateOdometry(eq_avg);
    }

    last_G_dis = G_dis;
}
//到时候看一下左右电机哪个对应 pwm_1
void set_pwm(int16_t pwm_l,int16_t pwm_r)
{   
    // 与速度环 Max_output 同一常数 MOTOR_PWM_LIMIT
    const int16_t lim = (int16_t)MOTOR_PWM_LIMIT;
    if (pwm_l > lim) pwm_l = lim;
    if (pwm_l < -lim) pwm_l = -lim;
    if (pwm_r > lim) pwm_r = lim;
    if (pwm_r < -lim) pwm_r = -lim;

    if(pwm_l>0)//正转
    {
        // 原因：左轮前进 DIR=0（原为 1，与本车/Car_Round 相反）。
        drv8701e_dir_1.set_level(0);
        drv8701e_pwm_1.set_duty(pwm_l);
    }
    else if(pwm_l<=0)
    {
        drv8701e_dir_1.set_level(1);
        drv8701e_pwm_1.set_duty(-pwm_l);
    }
    if(pwm_r>0)
    {
        drv8701e_dir_2.set_level(0);                                      // DIR输出高电平
        drv8701e_pwm_2.set_duty(pwm_r);       // 计算占空比
    }
    else if(pwm_r<=0)
    {
        drv8701e_dir_2.set_level(1);
        drv8701e_pwm_2.set_duty(-pwm_r);  
    }
}
