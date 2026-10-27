#include "zf_common_headfile.hpp"
#include "vofa_telemetry.h"
#include "run_mode.h"
#include <chrono>
#include <cstring>
#include <cstdio>

float g_vofa_snapshot[VOFA_CHANNEL_COUNT] = {0};
volatile int g_vofa_snapshot_ready = 0;

static zf_driver_tcp_client s_vofa_tcp;
static int s_vofa_tcp_ok = 0;
static auto s_last_send = std::chrono::steady_clock::now();
static const int s_send_period_ms = 10; // ~100Hz 上限，主循环再限

// ch[10]=Image_out；rawref=-Image_out
// ch[11]=g_middle_reference（真中环 r；slew=0 时=rawref）
// ch[15]=e=r-Dis_Speed；ch[16]=ΔP；ch[17]=ΔI；ch[18]=Dout；ch[19]=Δu
static const char *s_csv_header =
    "control_tick,control_dt_ms,run_mode,image_frame_id,image_age_ms,image_period_ms,"
    "forward_row,Dir_err,Image_Kp_active,Image_Kd_active,Image_out,middle_reference,"
    "Tpm_Dis,G_dis,Dis_Speed,middle_error,middle_dP,middle_dI,Dis_Out,middle_dU,"
    "speed_goal,left_target,left_feedback,left_error,left_ff,left_p,left_i,left_integral,PWM_L,"
    "right_target,right_feedback,right_error,right_ff,right_p,right_i,right_integral,PWM_R,"
    "saturation_bits,shizi_state,Huandao_L,Huandao_R,zebra_state,picture_state,"
    "small_rock_state,ramp_state,img_top,jump_point,dl1x_distance_raw";

const char *Vofa_CsvHeader(void)
{
    return s_csv_header;
}

void Vofa_FillSnapshot(void)
{
    float ch[VOFA_CHANNEL_COUNT] = {0};

    // 审查：直接读控制器本拍真分量，不重算
    const float left_ff = Velocity_L.ff_term;
    const float left_p = Velocity_L.p_term;
    const float left_i = Velocity_L.i_term;
    const float right_ff = Velocity_R.ff_term;
    const float right_p = Velocity_R.p_term;
    const float right_i = Velocity_R.i_term;

    ch[0] = (float)g_control_tick;
    ch[1] = g_control_dt_ms;
    ch[2] = (float)g_run_mode;
    ch[3] = (float)g_image_frame_id;
    ch[4] = g_image_age_ms;
    ch[5] = 0; // image_period_ms：后续可补帧间差
    ch[6] = (float)forward1;
    ch[7] = Dir_err;
    ch[8] = Image.Kp;
    ch[9] = Image.Kd;
    ch[10] = Image_out;
    ch[11] = g_middle_reference; // 真中环目标 mref（变化率限制后）
    ch[12] = Tpm_Dis;
    ch[13] = G_dis;
    ch[14] = Dis_Speed;
    ch[15] = Dis_1.Error;      // e=r-Dis_Speed
    ch[16] = g_middle_delta_p; // Kp*(e-e_last)
    ch[17] = g_middle_delta_i; // Ki*e（抗饱和后）
    ch[18] = Dis_Out;          // 有记忆的增量输出
    ch[19] = g_middle_delta;   // 本拍 Δu
    ch[20] = speed_goal;
    ch[21] = v_left_target;
    ch[22] = encoder_L.speed;
    ch[23] = Velocity_L.error;
    ch[24] = left_ff;
    ch[25] = left_p;
    ch[26] = left_i;
    ch[27] = Velocity_L.Integral;
    ch[28] = (float)PWM_L;
    ch[29] = v_right_target;
    ch[30] = encoder_R.speed;
    ch[31] = Velocity_R.error;
    ch[32] = right_ff;
    ch[33] = right_p;
    ch[34] = right_i;
    ch[35] = Velocity_R.Integral;
    ch[36] = (float)PWM_R;
    ch[37] = (float)(Velocity_L.sat_flags | (Velocity_R.sat_flags << 4));
    ch[38] = (float)Flag.Shizi;
    ch[39] = (float)Flag.Huandao_L;
    ch[40] = (float)Flag.Huandao_R;
    ch[41] = (float)Flag.Zebra_cross;
    ch[42] = (float)Flag.picture;
    ch[43] = (float)Flag.small_rock;
    ch[44] = (float)Flag.ramp;
    ch[45] = (float)imgInfo.top;
    ch[46] = (float)jump_point;
    ch[47] = (float)dl1x_distance_raw;

    memcpy(g_vofa_snapshot, ch, sizeof(ch));
    g_vofa_snapshot_ready = 1;
}

void Vofa_InitTcp(const char *ip, int port)
{
    // 原因：仅 OUTPUT_MODE_VOFA 时连接；失败只打日志，不改控制。
    if (s_vofa_tcp.init(ip, port) == 0)
    {
        s_vofa_tcp_ok = 1;
        printf("[VOFA] TCP connected %s:%d\n", ip, port);
    }
    else
    {
        s_vofa_tcp_ok = 0;
        printf("[VOFA] TCP connect failed %s:%d (control continues)\n", ip, port);
    }
}

void Vofa_TrySendFromMain(void)
{
    if (g_output_mode != OUTPUT_MODE_VOFA || !s_vofa_tcp_ok || !g_vofa_snapshot_ready)
        return;

    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - s_last_send).count() < s_send_period_ms)
        return;
    s_last_send = now;

    // JustFloat：48 * float32 + 帧尾 00 00 80 7f
    uint8_t frame[VOFA_CHANNEL_COUNT * 4 + 4];
    memcpy(frame, g_vofa_snapshot, VOFA_CHANNEL_COUNT * 4);
    frame[VOFA_CHANNEL_COUNT * 4 + 0] = 0x00;
    frame[VOFA_CHANNEL_COUNT * 4 + 1] = 0x00;
    frame[VOFA_CHANNEL_COUNT * 4 + 2] = 0x80;
    frame[VOFA_CHANNEL_COUNT * 4 + 3] = 0x7f;
    s_vofa_tcp.send_data(frame, sizeof(frame));
}

void Vofa_Close(void)
{
    s_vofa_tcp_ok = 0;
}
