// 对照 Car_Round/user/tools/key_supervisor.cpp 重组。
// 运行级按键：KEY1 发车+逐飞、KEY2 硬停、KEY3 发车无上位机、KEY4 锁电机看图。
// KEY2 必须：先 PWM=0，再 killall -9（不 waitpid，不堵轮询环）。
#include "zf_driver_gpio.hpp"
#include "zf_driver_pwm.hpp"
#include "upper_host_config.h"

#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
constexpr int k_poll_ms = 20;
constexpr int k_debounce_ticks = 3;
constexpr const char *k_default_bin = "/root/project";
constexpr const char *k_app_log = "/tmp/project.log";
constexpr const char *k_supervisor_log = "/tmp/key_supervisor.log";

void append_log(const char *fmt, ...)
{
    FILE *fp = std::fopen(k_supervisor_log, "a");
    if (fp == nullptr)
        return;
    std::time_t now = std::time(nullptr);
    std::tm tm_now = {};
    localtime_r(&now, &tm_now);
    std::fprintf(fp, "[%02d:%02d:%02d] ", tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
    va_list args;
    va_start(args, fmt);
    std::vfprintf(fp, fmt, args);
    va_end(args);
    std::fputc('\n', fp);
    std::fclose(fp);
}

bool is_char_device(const char *path)
{
    struct stat st = {};
    return path != nullptr && stat(path, &st) == 0 && S_ISCHR(st.st_mode);
}

bool wait_required_devices()
{
    static const char *required[] = {
        ZF_GPIO_KEY_1, ZF_GPIO_KEY_2, ZF_GPIO_KEY_3, ZF_GPIO_KEY_4, ZF_GPIO_BEEP,
        ZF_GPIO_MOTOR_1, ZF_GPIO_MOTOR_2, ZF_PWM_MOTOR_1, ZF_PWM_MOTOR_2,
    };
    for (int retry = 0; retry < 50; ++retry)
    {
        bool all_ready = true;
        for (const char *path : required)
        {
            if (!is_char_device(path))
            {
                all_ready = false;
                break;
            }
        }
        if (all_ready)
        {
            append_log("boot: zf device nodes ready");
            return true;
        }
        usleep(100 * 1000);
    }
    for (const char *path : required)
    {
        if (!is_char_device(path))
        {
            std::printf("device not ready: %s\n", path);
            append_log("device not ready: %s", path);
        }
    }
    return false;
}

zf_driver_gpio &key1_dev()
{
    static zf_driver_gpio dev(ZF_GPIO_KEY_1, O_RDWR);
    return dev;
}
zf_driver_gpio &key2_dev()
{
    static zf_driver_gpio dev(ZF_GPIO_KEY_2, O_RDWR);
    return dev;
}
zf_driver_gpio &key3_dev()
{
    static zf_driver_gpio dev(ZF_GPIO_KEY_3, O_RDWR);
    return dev;
}
zf_driver_gpio &key4_dev()
{
    static zf_driver_gpio dev(ZF_GPIO_KEY_4, O_RDWR);
    return dev;
}
zf_driver_gpio &beep_dev()
{
    static zf_driver_gpio dev(ZF_GPIO_BEEP, O_RDWR);
    return dev;
}
zf_driver_gpio &motor_dir_1_dev()
{
    static zf_driver_gpio dev(ZF_GPIO_MOTOR_1, O_RDWR);
    return dev;
}
zf_driver_gpio &motor_dir_2_dev()
{
    static zf_driver_gpio dev(ZF_GPIO_MOTOR_2, O_RDWR);
    return dev;
}
zf_driver_pwm &motor_pwm_1_dev()
{
    static zf_driver_pwm dev(ZF_PWM_MOTOR_1);
    return dev;
}
zf_driver_pwm &motor_pwm_2_dev()
{
    static zf_driver_pwm dev(ZF_PWM_MOTOR_2);
    return dev;
}

bool is_pressed(zf_driver_gpio &key)
{
    return !key.get_level();
}

int run_shell(const char *cmd)
{
    if (cmd == nullptr || cmd[0] == '\0')
        return -1;
    const int ok = std::system(cmd);
    if (ok != 0)
    {
        std::printf("command failed(%d): %s\n", ok, cmd);
        append_log("command failed(%d): %s", ok, cmd);
    }
    return ok;
}

void short_beep(int ms)
{
    beep_dev().set_level(1);
    usleep(ms * 1000);
    beep_dev().set_level(0);
}

// 与 Car_Round 相同：立刻 PWM=0 + DIR 安全电平，不进控制进程
void stop_motor_now()
{
    motor_pwm_1_dev().set_duty(0);
    motor_pwm_2_dev().set_duty(0);
    motor_dir_1_dev().set_level(0);
    motor_dir_2_dev().set_level(0);
}

// KEY2 核心：killall -9，绝不 waitpid 阻塞
void kill_project()
{
    run_shell("killall -9 project 2>/dev/null || true");
}

// menu_flag / output: 0=none 1=vofa 2=assistant（本工程 CLI 用字符串）
// 参数文件为绝对路径 /root/params_config.txt，与 cwd 无关；仍 cd /root 便于相对二进制路径。
void start_project(const char *bin, int menu_flag, const char *output_mode, const char *mode_name)
{
    kill_project();
    stop_motor_now();

    char cmd[1536] = {};
    // 用 nohup 后台，不占 supervisor 的 wait
    // OUTPUT：none / vofa / assistant
    int n = 0;
    if (std::strcmp(output_mode, "vofa") == 0)
    {
        n = std::snprintf(cmd, sizeof(cmd),
                          "cd /root && RUN_MODE=%d OUTPUT_MODE=vofa "
                          "nohup %s %d --vofa >%s 2>&1 &",
                          menu_flag, bin, menu_flag, k_app_log);
    }
    else if (std::strcmp(output_mode, "assistant") == 0)
    {
        n = std::snprintf(cmd, sizeof(cmd),
                          "cd /root && RUN_MODE=%d OUTPUT_MODE=assistant "
                          "ASSISTANT_DIV=5 ASSISTANT_SEND_IMAGE=1 "
                          "nohup %s %d --assistant >%s 2>&1 &",
                          menu_flag, bin, menu_flag, k_app_log);
    }
    else
    {
        n = std::snprintf(cmd, sizeof(cmd),
                          "cd /root && RUN_MODE=%d OUTPUT_MODE=none "
                          "nohup %s %d --none >%s 2>&1 &",
                          menu_flag, bin, menu_flag, k_app_log);
    }

    if (n < 0 || n >= static_cast<int>(sizeof(cmd)))
    {
        append_log("start failed: command truncated");
        return;
    }

    append_log("start: mode=%s menu=%d output=%s bin=%s", mode_name, menu_flag, output_mode, bin);
    std::printf("start: %s menu=%d output=%s\n", mode_name, menu_flag, output_mode);
    run_shell(cmd);
}

struct key_debounce_t
{
    bool last_pressed;
    int ticks;
};

// 仅在「按下边沿」且去抖后返回 true（与 Car_Round 一致）
bool pressed_edge(zf_driver_gpio &key, key_debounce_t *st)
{
    const bool now = is_pressed(key);
    if (now == st->last_pressed)
    {
        st->ticks = 0;
        return false;
    }
    ++st->ticks;
    if (st->ticks < k_debounce_ticks)
        return false;
    st->last_pressed = now;
    st->ticks = 0;
    return now; // 只在变为按下时 true
}

} // namespace

int main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::setvbuf(stderr, nullptr, _IOLBF, 0);

    const char *bin = k_default_bin;
    if (argc > 1 && argv[1] != nullptr && argv[1][0] != '\0')
        bin = argv[1];

    std::printf("key_supervisor: KEY1=run(10,assistant) KEY2=stop KEY3=run(10,none) KEY4=debug(30,assistant)\n");
    std::printf("bin=%s default_upper=%s:%d log=%s\n",
                bin, UPPER_HOST_IP, UPPER_HOST_PORT, k_supervisor_log);
    append_log("boot: key_supervisor started bin=%s", bin);

    if (!wait_required_devices())
    {
        append_log("boot: zf device check failed, exit");
        return 1;
    }

    // 预热 PWM 节点，保证 KEY2 能立刻清零
    struct pwm_info info;
    motor_pwm_1_dev().get_dev_info(&info);
    motor_pwm_2_dev().get_dev_info(&info);
    beep_dev().set_level(0);
    stop_motor_now();

    key_debounce_t k1 = {is_pressed(key1_dev()), 0};
    key_debounce_t k2 = {is_pressed(key2_dev()), 0};
    key_debounce_t k3 = {is_pressed(key3_dev()), 0};
    key_debounce_t k4 = {is_pressed(key4_dev()), 0};
    append_log("boot: initial key k1=%d k2=%d k3=%d k4=%d",
               k1.last_pressed ? 1 : 0, k2.last_pressed ? 1 : 0,
               k3.last_pressed ? 1 : 0, k4.last_pressed ? 1 : 0);

    while (true)
    {
        // KEY1：巡线 + 逐飞助手。速度读取 params 的常规及 picture/ramp 状态预设。
        if (pressed_edge(key1_dev(), &k1))
        {
            std::printf("KEY1: start track assistant\n");
            append_log("event: KEY1 -> track 10 assistant");
            short_beep(80);
            start_project(bin, 10, "assistant", "track_assistant");
        }

        // KEY2：硬停 —— 先电机 0，再 killall -9，再电机 0；禁止 waitpid
        if (pressed_edge(key2_dev(), &k2))
        {
            std::printf("KEY2: hard stop\n");
            append_log("event: KEY2 -> hard stop");
            stop_motor_now();
            kill_project();
            short_beep(180);
            stop_motor_now();
        }

        // KEY3：巡线无上位机（原 KEY1=none）。
        // 原 show.cpp key_scan：KEY3(上) 为 Parameter_flag-- 菜单上翻，supervisor 不沿用该功能：
        // if (pressed_edge(...)) { Parameter_flag--; if (Parameter_flag < 1) Parameter_flag = 1; }
        if (pressed_edge(key3_dev(), &k3))
        {
            std::printf("KEY3: start track none\n");
            append_log("event: KEY3 -> track 10 none");
            short_beep(80);
            start_project(bin, 10, "none", "track_no_output");
        }

        // KEY4：锁电机图像 + 逐飞 V2（对齐安全调试；Car_Round 用 VOFA，本工程看图用 assistant）
        if (pressed_edge(key4_dev(), &k4))
        {
            std::printf("KEY4: image debug assistant\n");
            append_log("event: KEY4 -> image 30 assistant");
            short_beep(80);
            usleep(80 * 1000);
            short_beep(80);
            start_project(bin, 30, "assistant", "safe_debug_assistant");
        }

        usleep(k_poll_ms * 1000);
    }
}
