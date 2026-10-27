// 原因：输出 raw XYZ 与车辆 yaw 候选（raw Y 减偏后取负，左转+，rad/s）。默认不驱电机。
#include "zf_device_imu.hpp"
#include "zf_driver_delay.hpp"
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static volatile int g_run = 1;
static void on_sig(int) { g_run = 0; }

int main(int argc, char **argv)
{
    signal(SIGINT, on_sig);
    int period_ms = 20;
    if (argc > 1)
        period_ms = atoi(argv[1]);
    if (period_ms < 5)
        period_ms = 5;

    zf_device_imu imu;
    if (imu.init() == DEV_NO_FIND)
    {
        printf("imu init failed (DEV_NO_FIND)\n");
        return 1;
    }

    // 静止零偏采样
    const int N = 200;
    double bx = 0, by = 0, bz = 0;
    printf("imu_watch: sampling bias %d samples...\n", N);
    for (int i = 0; i < N; i++)
    {
        bx += imu.get_gyro_x();
        by += imu.get_gyro_y();
        bz += imu.get_gyro_z();
        system_delay_ms(5);
    }
    bx /= N;
    by /= N;
    bz /= N;
    printf("bias gyro raw: x=%.2f y=%.2f z=%.2f\n", bx, by, bz);
    printf("t_ms,dt_ms,raw_gx,raw_gy,raw_gz,raw_ax,raw_ay,raw_az,yaw_rate_rad_s\n");

    auto t0 = std::chrono::steady_clock::now();
    auto last = t0;
    // 2000dps -> 16.4 LSB/(deg/s) 与主工程一致
    const double raw_to_rad = (M_PI / 180.0) / 16.4;

    while (g_run)
    {
        auto now = std::chrono::steady_clock::now();
        double t_ms = std::chrono::duration<double, std::milli>(now - t0).count();
        double dt_ms = std::chrono::duration<double, std::milli>(now - last).count();
        last = now;

        double gx = imu.get_gyro_x();
        double gy = imu.get_gyro_y();
        double gz = imu.get_gyro_z();
        double ax = imu.get_acc_x();
        double ay = imu.get_acc_y();
        double az = imu.get_acc_z();
        // Car_Round：vehicle_yaw = -(raw_y - bias) * scale，左转 +
        double yaw_rate = -(gy - by) * raw_to_rad;

        printf("%.1f,%.2f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.5f\n",
               t_ms, dt_ms, gx, gy, gz, ax, ay, az, yaw_rate);
        fflush(stdout);
        system_delay_ms(period_ms);
    }
    return 0;
}
