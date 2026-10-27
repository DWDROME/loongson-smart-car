// 原因：独立架空验电机方向；固定小占空比 + 超时停 + 退出清零。不进主程序 aux。
#include "zf_driver_pwm.hpp"
#include "zf_driver_gpio.hpp"
#include "zf_driver_delay.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>

static zf_driver_pwm pwm_l(ZF_PWM_MOTOR_1);
static zf_driver_pwm pwm_r(ZF_PWM_MOTOR_2);
static zf_driver_gpio dir_l(ZF_GPIO_MOTOR_1, O_RDWR);
static zf_driver_gpio dir_r(ZF_GPIO_MOTOR_2, O_RDWR);

static void all_stop(void)
{
    pwm_l.set_duty(0);
    pwm_r.set_duty(0);
}

static void on_sig(int)
{
    all_stop();
    _exit(0);
}

// 与主程序 set_pwm 合同一致：正转 DIR=0；PWM 量程同主程序 ±5000。
// 标定用软顶 2000，避免误给满幅；死区常 >800（旧 cap 会导致“全程没动”）。
static void drive(int side, int duty_signed, int ms)
{
    const int lim = 2000;
    int d = duty_signed;
    if (d > lim) d = lim;
    if (d < -lim) d = -lim;

    auto &pwm = (side == 0) ? pwm_l : pwm_r;
    auto &dir = (side == 0) ? dir_l : dir_r;
    if (d > 0)
    {
        dir.set_level(0);
        pwm.set_duty((uint16)d);
    }
    else if (d < 0)
    {
        dir.set_level(1);
        pwm.set_duty((uint16)(-d));
    }
    else
    {
        pwm.set_duty(0);
    }
    system_delay_ms(ms);
    all_stop();
}

int main(int argc, char **argv)
{
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    printf("motor_spin: left|right|both  duty(+fwd -rev)  ms  (duty cap 2000)\n");
    if (argc < 4)
    {
        printf("usage: %s left|right|both <duty> <ms>\n", argv[0]);
        return 1;
    }
    const char *which = argv[1];
    int duty = atoi(argv[2]);
    int ms = atoi(argv[3]);
    if (ms <= 0) ms = 500;
    if (ms > 3000) ms = 3000;

    struct pwm_info info;
    pwm_l.get_dev_info(&info);
    pwm_r.get_dev_info(&info);

    if (strcmp(which, "left") == 0)
        drive(0, duty, ms);
    else if (strcmp(which, "right") == 0)
        drive(1, duty, ms);
    else if (strcmp(which, "both") == 0)
    {
        // 同时正转；cap 与 drive 一致
        if (duty > 2000) duty = 2000;
        if (duty < -2000) duty = -2000;
        if (duty > 0)
        {
            dir_l.set_level(0);
            dir_r.set_level(0);
            pwm_l.set_duty((uint16)duty);
            pwm_r.set_duty((uint16)duty);
        }
        else if (duty < 0)
        {
            dir_l.set_level(1);
            dir_r.set_level(1);
            pwm_l.set_duty((uint16)(-duty));
            pwm_r.set_duty((uint16)(-duty));
        }
        system_delay_ms(ms);
        all_stop();
    }
    else
    {
        printf("bad side\n");
        return 1;
    }
    printf("done, motors off\n");
    return 0;
}
