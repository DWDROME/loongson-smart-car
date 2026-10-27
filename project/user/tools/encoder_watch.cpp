// 原因：观察左右 QUAD 周期 count；左 QUAD2 前进+，右 QUAD1 取负后前进+。默认不驱电机。
#include "zf_driver_encoder.hpp"
#include "zf_driver_delay.hpp"
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstdint>

static volatile int g_run = 1;
static void on_sig(int) { g_run = 0; }

int main(int argc, char **argv)
{
    signal(SIGINT, on_sig);
    int period_ms = 50;
    if (argc > 1)
        period_ms = atoi(argv[1]);
    if (period_ms < 10)
        period_ms = 10;

    // 与主程序合同一致
    zf_driver_encoder enc_l(ZF_ENCODER_QUAD_2);
    zf_driver_encoder enc_r(ZF_ENCODER_QUAD_1);
    int64_t sum_l = 0, sum_r = 0;

    printf("encoder_watch: L=QUAD2 R=QUAD1(-) period=%dms  Ctrl+C quit\n", period_ms);
    printf("tick,count_l,count_r,sum_l,sum_r\n");
    int tick = 0;
    while (g_run)
    {
        int16_t cl = enc_l.get_count();
        int16_t cr = (int16_t)(-enc_r.get_count());
        enc_l.clear_count();
        enc_r.clear_count();
        sum_l += cl;
        sum_r += cr;
        printf("%d,%d,%d,%lld,%lld\n", tick++, (int)cl, (int)cr,
               (long long)sum_l, (long long)sum_r);
        fflush(stdout);
        system_delay_ms(period_ms);
    }
    return 0;
}
