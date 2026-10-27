#include "zf_common_headfile.hpp"
#include "detour/red_brick_detour.hpp"
#include "detour/red_event_router.hpp"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef PARAM_FILE_NAME
#define PARAM_FILE_NAME "/root/params_config.txt"
#endif

FlashInformation Flash = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .small_err = 15,
    .picture_err = 30,
    .forward = 24,
    .debug_rgb_r_min = 120,
    .debug_rgb_rb_diff = 35,
    .debug_rgb_rg_diff = 35,
    .camera_exposure = -1
};

// 文件加载缓存（控制键）；默认与 Direction_PID_Init / ae3379a 中环对齐
static float s_speed_kf_l = 8.0f, s_speed_kp_l = 30.0f, s_speed_ki_l = 0.0f;
static float s_speed_kf_r = 8.0f, s_speed_kp_r = 30.0f, s_speed_ki_r = 0.0f;
static float s_speed_max_i = 1500.0f, s_speed_max_out = 5000.0f, s_speed_max_error = 2000.0f;
// 中环 ae3379a 增量 PI：Kp=0.7 Ki=0.3 Kd=0；max=70 为执行安全边界（作者曾 9999）
static float s_middle_kp = 0.70f, s_middle_kd = 0.0f, s_middle_ki = 0.30f;
static float s_middle_max_out = 70.0f;
static float s_middle_ref_slew = 0.0f;   // 0=直通（正式默认；不挡急弯参考）
static float s_middle_tpm_weight = 0.0f; // 0=纯 G；0.1 表示 0.9G+0.1Tpm
static float s_image_kp_scale = 1.0f, s_image_kd = 2.0f;
static float s_threshold_static = 35.0f; // 大津二值下限，写入 Threshold_static
static float s_camera_exposure = -1.0f;  // -1=driver default, 0=explicit auto, [20,200]=manual
static float s_base_speed = 0.0f; // 安全默认 0，标定后再 remote 写入
static float s_straight_trim = 0.025f;
static float s_picture_speed = 100.0f; // ae3379a picture==2 速度预设
static float s_ramp_speed = 150.0f;    // ae3379a ramp==2 速度预设
// 图片识别距离默认值等于 image.cpp 原硬编码；目标板分类后统一直行。
static float s_picture_recog_k = 0.1f;
static float s_picture_recog_base = 40.0f;
static float s_picture_near_dist = 30.0f;
static float s_picture_side_offset = 10.0f;
static float s_picture_hold_dist = 30.0f;
static float s_picture_finish_dist = 40.0f;
static float s_picture_conf_min = 0.50f;
static red_event::RedEventRouterConfig s_applied_red_router_config{};
static red_event::RedBrickDetourConfig s_applied_red_detour_config{};
static float s_red_brick_enable = s_applied_red_router_config.brick_enable ? 1.0f : 0.0f;
static float s_picture_roi_enable = s_applied_red_router_config.picture_roi_enable ? 1.0f : 0.0f;
static float s_red_route_white_ratio = s_applied_red_router_config.white_ratio;
static float s_red_route_confirm_frames = s_applied_red_router_config.confirm_frames;
static float s_red_brick_side_deadband = s_applied_red_router_config.side_deadband;
static float s_red_brick_slow_speed = s_applied_red_detour_config.slow_speed;
static float s_red_brick_speed = s_applied_red_detour_config.past_speed;
static float s_red_brick_offset_out = s_applied_red_detour_config.offset_out;
static float s_red_brick_offset_back = s_applied_red_detour_config.offset_back;
static float s_red_brick_slowdown_ms = s_applied_red_detour_config.slowdown_ms;
static float s_red_brick_out_ms = s_applied_red_detour_config.out_ms;
static float s_red_brick_past_ms = s_applied_red_detour_config.past_ms;
static float s_red_brick_back_ms = s_applied_red_detour_config.back_ms;
static float s_red_brick_recover_ms = s_applied_red_detour_config.recover_ms;
static float s_forward_base = 30.0f;
static float s_huandao_l_state5_exit_yaw = 25.0f;
static float s_huandao_r_state5_exit_yaw = -55.0f;
static float s_middle_test_target = 0.0f;
static float s_count_per_meter = 35000.0f;
static float s_count_per_meter_l = 0.0f; // 0 = 继承 count_per_meter
static float s_count_per_meter_r = 0.0f;
static float s_track_width_m = 0.15f;
static float s_c_gyro = 28.144800533f;
static int s_param_apply_ok = 1;

enum RedParamIndex
{
    kRedBrickEnable = 0,
    kPictureRoiEnable,
    kRedRouteWhiteRatio,
    kRedRouteConfirmFrames,
    kRedBrickSideDeadband,
    kRedBrickSlowSpeed,
    kRedBrickSpeed,
    kRedBrickOffsetOut,
    kRedBrickOffsetBack,
    kRedBrickSlowdownMs,
    kRedBrickOutMs,
    kRedBrickPastMs,
    kRedBrickBackMs,
    kRedBrickRecoverMs,
    kRedParamCount,
};

static const char *const s_red_param_names[kRedParamCount] = {
    "red_brick_enable",
    "picture_roi_enable",
    "red_route_white_ratio",
    "red_route_confirm_frames",
    "red_brick_side_deadband",
    "red_brick_slow_speed",
    "red_brick_speed",
    "red_brick_offset_out",
    "red_brick_offset_back",
    "red_brick_slowdown_ms",
    "red_brick_out_ms",
    "red_brick_past_ms",
    "red_brick_back_ms",
    "red_brick_recover_ms",
};
static int s_red_param_found[kRedParamCount] = {0};

static int parse_line(const char* line, const char* target_key, float* out_value) {
    char key_buf[32] = {0};
    float val_buf = 0.0f;
    char trailing = 0;
    if (sscanf(line, "%31[^=]=%f %c", key_buf, &val_buf, &trailing) == 2) {
        char* p = key_buf;
        while(*p) {
            if(*p == '\r' || *p == '\n') *p = 0;
            p++;
        }
        if (strcmp(key_buf, target_key) == 0) {
            *out_value = val_buf;
            return 1;
        }
    }
    return 0;
}

static int line_has_key(const char *line, const char *target_key)
{
    char key_buf[32] = {0};
    if (sscanf(line, "%31[^=]=", key_buf) != 1)
        return 0;
    return strcmp(key_buf, target_key) == 0;
}

static void try_load(const char* line, const char* key, float* dst, int* found_count)
{
    float temp = 0.0f;
    if (parse_line(line, key, &temp)) {
        *dst = temp;
        (*found_count)++;
    }
}

static void try_load_red(const char *line, RedParamIndex index, float *dst,
                         int *found_count)
{
    float temp = 0.0f;
    if (line_has_key(line, s_red_param_names[index]))
    {
        s_red_param_found[index]++;
        (*found_count)++;
        if (parse_line(line, s_red_param_names[index], &temp))
            *dst = temp;
        else
            *dst = NAN;
    }
}

static int IsIntegerValue(float value)
{
    return isfinite((double)value) && floorf(value) == value;
}

// 返回 0 通过；非 0 失败（不写控制器）。审查：isfinite + 合同边界。
static int Param_ValidateStaging(void)
{
    const float *need_finite[] = {
        &s_speed_kf_l, &s_speed_kp_l, &s_speed_ki_l,
        &s_speed_kf_r, &s_speed_kp_r, &s_speed_ki_r,
        &s_speed_max_i, &s_speed_max_out, &s_speed_max_error,
        &s_middle_kp, &s_middle_kd, &s_middle_ki, &s_middle_max_out,
        &s_middle_ref_slew, &s_middle_tpm_weight,
        &s_image_kp_scale, &s_image_kd, &s_threshold_static,
        &s_camera_exposure,
        &s_base_speed, &s_straight_trim, &s_picture_speed, &s_ramp_speed,
        &s_picture_recog_k, &s_picture_recog_base, &s_picture_near_dist,
        &s_picture_side_offset, &s_picture_hold_dist, &s_picture_finish_dist,
        &s_picture_conf_min,
        &s_red_brick_enable, &s_picture_roi_enable,
        &s_red_route_white_ratio, &s_red_route_confirm_frames,
        &s_red_brick_side_deadband, &s_red_brick_slow_speed, &s_red_brick_speed,
        &s_red_brick_offset_out, &s_red_brick_offset_back,
        &s_red_brick_slowdown_ms, &s_red_brick_out_ms, &s_red_brick_past_ms,
        &s_red_brick_back_ms, &s_red_brick_recover_ms,
        &s_forward_base,
        &s_huandao_l_state5_exit_yaw, &s_huandao_r_state5_exit_yaw,
        &s_middle_test_target, &s_count_per_meter, &s_count_per_meter_l, &s_count_per_meter_r,
        &s_track_width_m, &s_c_gyro,
    };
    for (unsigned i = 0; i < sizeof(need_finite) / sizeof(need_finite[0]); ++i)
    {
        if (!isfinite((double)(*need_finite[i])))
        {
            printf("[Param] REJECT: non-finite value at field index %u\r\n", i);
            return -1;
        }
    }
    if (s_speed_kf_l < 0.0f || s_speed_kp_l < 0.0f || s_speed_ki_l < 0.0f ||
        s_speed_kf_r < 0.0f || s_speed_kp_r < 0.0f || s_speed_ki_r < 0.0f)
    {
        printf("[Param] REJECT: speed gains must be >= 0\r\n");
        return -1;
    }
    if (!(s_speed_max_out > 0.0f) || s_speed_max_out > MOTOR_PWM_LIMIT)
    {
        printf("[Param] REJECT: speed_max_out must be in (0, %.0f]\r\n", MOTOR_PWM_LIMIT);
        return -1;
    }
    if (!(s_middle_max_out > 0.0f))
    {
        printf("[Param] REJECT: middle_max_out must be > 0\r\n");
        return -1;
    }
    // middle_ref_slew：0=直通；>0 为 G/s
    if (s_middle_ref_slew < 0.0f || s_middle_ref_slew > 20000.0f)
    {
        printf("[Param] REJECT: middle_ref_slew must be in [0,20000]\r\n");
        return -1;
    }
    if (s_middle_tpm_weight < 0.0f || s_middle_tpm_weight > 1.0f)
    {
        printf("[Param] REJECT: middle_tpm_weight must be in [0,1]\r\n");
        return -1;
    }
    if (s_middle_ki < 0.0f || s_middle_kp < 0.0f)
    {
        printf("[Param] REJECT: middle_kp/ki must be >= 0\r\n");
        return -1;
    }
    if (s_base_speed < 0.0f || s_base_speed > 2000.0f)
    {
        printf("[Param] REJECT: base_speed out of [0,2000]\r\n");
        return -1;
    }
    if (s_straight_trim < -0.10f || s_straight_trim > 0.10f)
    {
        printf("[Param] REJECT: straight_trim must be in [-0.10,0.10]\r\n");
        return -1;
    }
    if (!(s_picture_speed > 0.0f) || s_picture_speed > 2000.0f ||
        !(s_ramp_speed > 0.0f) || s_ramp_speed > 2000.0f)
    {
        printf("[Param] REJECT: picture_speed/ramp_speed presets must be in (0,2000]\r\n");
        return -1;
    }
    if (s_picture_recog_k < 0.0f || s_picture_recog_k > 2.0f ||
        s_picture_recog_base < 0.0f || s_picture_recog_base > 500.0f)
    {
        printf("[Param] REJECT: picture_recog_k/base out of range\r\n");
        return -1;
    }
    if (s_picture_near_dist < 1.0f || s_picture_near_dist > 500.0f ||
        s_picture_hold_dist < 1.0f || s_picture_hold_dist > 500.0f ||
        s_picture_finish_dist < 1.0f || s_picture_finish_dist > 500.0f)
    {
        printf("[Param] REJECT: picture near/hold/finish distance out of [1,500]\r\n");
        return -1;
    }
    if (s_picture_side_offset < 0.0f || s_picture_side_offset > 40.0f)
    {
        printf("[Param] REJECT: picture_side_offset must be in [0,40]\r\n");
        return -1;
    }
    if (s_picture_conf_min < 0.0f || s_picture_conf_min > 1.0f)
    {
        printf("[Param] REJECT: picture_conf_min must be in [0,1]\r\n");
        return -1;
    }
    for (int i = 0; i < kRedParamCount; ++i)
    {
        if (s_red_param_found[i] != 1)
        {
            printf("[Param] REJECT: %s must appear exactly once (found=%d)\r\n",
                   s_red_param_names[i], s_red_param_found[i]);
            return -1;
        }
    }
    if (!IsIntegerValue(s_red_brick_enable) ||
        (s_red_brick_enable != 0.0f && s_red_brick_enable != 1.0f))
    {
        printf("[Param] REJECT: red_brick_enable must be integer 0 or 1\r\n");
        return -1;
    }
    if (!IsIntegerValue(s_picture_roi_enable) ||
        (s_picture_roi_enable != 0.0f && s_picture_roi_enable != 1.0f))
    {
        printf("[Param] REJECT: picture_roi_enable must be integer 0 or 1\r\n");
        return -1;
    }
    if (s_red_route_white_ratio < 0.0f || s_red_route_white_ratio > 1.0f)
    {
        printf("[Param] REJECT: red_route_white_ratio must be in [0,1]\r\n");
        return -1;
    }
    if (!IsIntegerValue(s_red_route_confirm_frames) ||
        s_red_route_confirm_frames < 1.0f || s_red_route_confirm_frames > 10.0f)
    {
        printf("[Param] REJECT: red_route_confirm_frames must be integer [1,10]\r\n");
        return -1;
    }
    if (!IsIntegerValue(s_red_brick_side_deadband) ||
        s_red_brick_side_deadband < 0.0f || s_red_brick_side_deadband > 30.0f)
    {
        printf("[Param] REJECT: red_brick_side_deadband must be integer [0,30]\r\n");
        return -1;
    }
    if (!(s_red_brick_slow_speed > 0.0f) || s_red_brick_slow_speed > 2000.0f ||
        !(s_red_brick_speed > 0.0f) || s_red_brick_speed > 2000.0f)
    {
        printf("[Param] REJECT: red brick speeds must be in (0,2000]\r\n");
        return -1;
    }
    if (s_red_brick_offset_out < 0.0f || s_red_brick_offset_out > 40.0f ||
        s_red_brick_offset_back < 0.0f || s_red_brick_offset_back > 40.0f)
    {
        printf("[Param] REJECT: red brick offsets must be in [0,40]\r\n");
        return -1;
    }
    const float red_durations[] = {
        s_red_brick_slowdown_ms, s_red_brick_out_ms, s_red_brick_past_ms,
        s_red_brick_back_ms, s_red_brick_recover_ms,
    };
    for (unsigned i = 0; i < sizeof(red_durations) / sizeof(red_durations[0]); ++i)
    {
        if (!IsIntegerValue(red_durations[i]) ||
            red_durations[i] < 1.0f || red_durations[i] > 10000.0f)
        {
            printf("[Param] REJECT: red brick durations must be integer [1,10000] ms\r\n");
            return -1;
        }
    }
    if (s_huandao_l_state5_exit_yaw < -180.0f || s_huandao_l_state5_exit_yaw > 180.0f ||
        s_huandao_r_state5_exit_yaw < -180.0f || s_huandao_r_state5_exit_yaw > 180.0f)
    {
        printf("[Param] REJECT: huandao state5 exit yaw must be in [-180,180]\r\n");
        return -1;
    }
    if (!(s_count_per_meter > 1.0f) || s_count_per_meter > 1.0e7f)
    {
        printf("[Param] REJECT: count_per_meter invalid\r\n");
        return -1;
    }
    // l/r 允许 0（表示继承均值）；若给出则须合法
    if (s_count_per_meter_l != 0.0f &&
        (!(s_count_per_meter_l > 1.0f) || s_count_per_meter_l > 1.0e7f))
    {
        printf("[Param] REJECT: count_per_meter_l invalid\r\n");
        return -1;
    }
    if (s_count_per_meter_r != 0.0f &&
        (!(s_count_per_meter_r > 1.0f) || s_count_per_meter_r > 1.0e7f))
    {
        printf("[Param] REJECT: count_per_meter_r invalid\r\n");
        return -1;
    }
    if (!(s_track_width_m > 0.05f) || s_track_width_m > 1.0f)
    {
        printf("[Param] REJECT: track_width_m invalid\r\n");
        return -1;
    }
    if (!(s_c_gyro > 0.0f) || s_c_gyro > 1.0e5f)
    {
        printf("[Param] REJECT: c_gyro invalid\r\n");
        return -1;
    }
    // 实际是 Kd*(D_speed_L-D_speed_R)，不是误差 D；未完成 dt/符号标定前锁 0。
    if (s_middle_kd != 0.0f)
    {
        printf("[Param] REJECT: middle_kd wheel-delta term is uncalibrated; keep 0\r\n");
        return -1;
    }
    // 大津二值下限：0~180（与 Threshold_deal 扫描上界一致）
    if (s_threshold_static < 0.0f || s_threshold_static > 180.0f)
    {
        printf("[Param] REJECT: threshold_static must be in [0,180]\r\n");
        return -1;
    }
    if (s_camera_exposure != -1.0f && s_camera_exposure != 0.0f &&
        (s_camera_exposure < 20.0f || s_camera_exposure > 200.0f))
    {
        printf("[Param] REJECT: camera_exposure must be -1(driver-default), 0(auto), or in [20,200]\r\n");
        return -1;
    }
    return 0;
}

void Param_ApplyToControllers(void)
{
    if (Param_ValidateStaging() != 0)
    {
        s_param_apply_ok = 0;
        printf("[Param] Apply aborted; controllers unchanged; keep base_speed=0 / STOP safe.\r\n");
        g_base_speed = 0.0f;
        return;
    }
    s_param_apply_ok = 1;

    const float speed_max = s_speed_max_out;
    const float mid_max = s_middle_max_out;

    Velocity_L.kf = s_speed_kf_l;
    Velocity_L.kp = s_speed_kp_l;
    Velocity_L.ki = s_speed_ki_l;
    Velocity_L.kd = 0;
    Velocity_L.Max_integral = s_speed_max_i;
    Velocity_L.Max_output = speed_max;
    Velocity_L.Max_error = s_speed_max_error;

    Velocity_R.kf = s_speed_kf_r;
    Velocity_R.kp = s_speed_kp_r;
    Velocity_R.ki = s_speed_ki_r;
    Velocity_R.kd = 0;
    Velocity_R.Max_integral = s_speed_max_i;
    Velocity_R.Max_output = speed_max;
    Velocity_R.Max_error = s_speed_max_error;

    Velocity.kf = 0.5f * (s_speed_kf_l + s_speed_kf_r);
    Velocity.kp = 0.5f * (s_speed_kp_l + s_speed_kp_r);
    Velocity.ki = 0.5f * (s_speed_ki_l + s_speed_ki_r);
    Velocity.kd = 0;
    Velocity.Max_integral = s_speed_max_i;
    Velocity.Max_output = speed_max;
    Velocity.Max_error = s_speed_max_error;

    Dis_1.Kp = s_middle_kp;
    Dis_1.Kd = s_middle_kd;
    Dis_1.Ki = s_middle_ki;
    Dis_1.MAX_OutPut = mid_max;
    g_middle_ref_slew_per_s = s_middle_ref_slew;
    LPF_Speed = s_middle_tpm_weight;

    g_image_kp_scale = s_image_kp_scale;
    Image.Kd = s_image_kd;
    // 注意：动态 Image.Kp 每帧覆盖；scale 不在此重复乘，避免复合倍增（审查 P2）
    (void)g_image_kp_scale;
    Threshold_static = (uint8)(s_threshold_static + 0.5f);
    Flash.camera_exposure = (s_camera_exposure < 0.0f)
                                ? -1
                                : (int)(s_camera_exposure + 0.5f);

    g_base_speed = s_base_speed;
    g_straight_trim = s_straight_trim;
    g_picture_speed = s_picture_speed;
    g_ramp_speed = s_ramp_speed;
    g_picture_recog_k = s_picture_recog_k;
    g_picture_recog_base = s_picture_recog_base;
    g_picture_near_dist = s_picture_near_dist;
    g_picture_side_offset = s_picture_side_offset;
    g_picture_hold_dist = s_picture_hold_dist;
    g_picture_finish_dist = s_picture_finish_dist;
    g_picture_conf_min = s_picture_conf_min;
    s_applied_red_router_config.white_ratio = s_red_route_white_ratio;
    s_applied_red_router_config.confirm_frames =
        static_cast<std::uint8_t>(s_red_route_confirm_frames);
    s_applied_red_router_config.side_deadband =
        static_cast<std::int16_t>(s_red_brick_side_deadband);
    s_applied_red_router_config.brick_enable = s_red_brick_enable != 0.0f;
    s_applied_red_router_config.picture_roi_enable = s_picture_roi_enable != 0.0f;
    s_applied_red_detour_config.slowdown_ms =
        static_cast<std::uint32_t>(s_red_brick_slowdown_ms);
    s_applied_red_detour_config.out_ms =
        static_cast<std::uint32_t>(s_red_brick_out_ms);
    s_applied_red_detour_config.past_ms =
        static_cast<std::uint32_t>(s_red_brick_past_ms);
    s_applied_red_detour_config.back_ms =
        static_cast<std::uint32_t>(s_red_brick_back_ms);
    s_applied_red_detour_config.recover_ms =
        static_cast<std::uint32_t>(s_red_brick_recover_ms);
    s_applied_red_detour_config.slow_speed = s_red_brick_slow_speed;
    s_applied_red_detour_config.past_speed = s_red_brick_speed;
    s_applied_red_detour_config.offset_out = s_red_brick_offset_out;
    s_applied_red_detour_config.offset_back = s_red_brick_offset_back;
    g_middle_test_target = s_middle_test_target;
    // forward_base 接入运行：TRACK 用 g_forward_base - Master_Speed/60
    g_forward_base = s_forward_base;
    g_huandao_l_state5_exit_yaw = s_huandao_l_state5_exit_yaw;
    g_huandao_r_state5_exit_yaw = s_huandao_r_state5_exit_yaw;
    forward1 = (int)s_forward_base;
    forward = forward1;

    g_count_per_meter_l =
        (s_count_per_meter_l > 1.0f) ? s_count_per_meter_l : s_count_per_meter;
    g_count_per_meter_r =
        (s_count_per_meter_r > 1.0f) ? s_count_per_meter_r : s_count_per_meter;
    g_count_per_meter = 0.5f * (g_count_per_meter_l + g_count_per_meter_r);
    // 回写均值，便于旧工具只认 count_per_meter
    s_count_per_meter = g_count_per_meter;
    g_track_width_m = s_track_width_m;
    g_c_gyro = s_c_gyro;
    Units_CgyroFitReset();

    printf("[Param] Applied: speedL kf/kp/ki=%.2f/%.2f/%.3f speedR=%.2f/%.2f/%.3f "
           "mid kp/ki/kd/max=%.2f/%.3f/%.2f/%.1f slew=%.0f tpm_w=%.4f base=%.1f trim=%.4f "
           "thr_min=%u camera_exp=%d picture/ramp=%.1f/%.1f "
           "pic k/base=%.2f/%.1f near/side/hold/finish=%.1f/%.1f/%.1f/%.1f conf=%.2f "
           "huandao5 L/R=%.1f/%.1f cpmL/R=%.0f/%.0f c_gyro=%.3f C_model=%.3f\n",
           Velocity_L.kf, Velocity_L.kp, Velocity_L.ki,
           Velocity_R.kf, Velocity_R.kp, Velocity_R.ki,
           Dis_1.Kp, Dis_1.Ki, Dis_1.Kd, Dis_1.MAX_OutPut,
           g_middle_ref_slew_per_s, LPF_Speed,
           g_base_speed, g_straight_trim, (unsigned)Threshold_static, Flash.camera_exposure,
           g_picture_speed, g_ramp_speed,
           g_picture_recog_k, g_picture_recog_base, g_picture_near_dist,
           g_picture_side_offset, g_picture_hold_dist, g_picture_finish_dist,
           g_picture_conf_min, g_huandao_l_state5_exit_yaw, g_huandao_r_state5_exit_yaw,
           g_count_per_meter_l, g_count_per_meter_r, g_c_gyro, Units_C_model());
    printf("[Param] Red route: brick/roi=%u/%u white_ratio=%.3f confirm=%u deadband=%d "
           "slow/past=%.1f/%.1f offset_out/back=%.1f/%.1f "
           "ms=%u/%u/%u/%u/%u\n",
           s_applied_red_router_config.brick_enable ? 1u : 0u,
           s_applied_red_router_config.picture_roi_enable ? 1u : 0u,
           s_applied_red_router_config.white_ratio,
           (unsigned)s_applied_red_router_config.confirm_frames,
           (int)s_applied_red_router_config.side_deadband,
           s_applied_red_detour_config.slow_speed,
           s_applied_red_detour_config.past_speed,
           s_applied_red_detour_config.offset_out,
           s_applied_red_detour_config.offset_back,
           (unsigned)s_applied_red_detour_config.slowdown_ms,
           (unsigned)s_applied_red_detour_config.out_ms,
           (unsigned)s_applied_red_detour_config.past_ms,
           (unsigned)s_applied_red_detour_config.back_ms,
           (unsigned)s_applied_red_detour_config.recover_ms);
}

const red_event::RedEventRouterConfig &Param_RedEventRouterConfig(void)
{
    return s_applied_red_router_config;
}

int Param_IsApplyOk(void)
{
    return s_param_apply_ok;
}

const red_event::RedBrickDetourConfig &Param_RedBrickDetourConfig(void)
{
    return s_applied_red_detour_config;
}

void Param_Init(void) {
    printf("[Param] Initializing from %s ...\r\n", PARAM_FILE_NAME);

    zf_driver_file_string file(PARAM_FILE_NAME, "r");
    char line_buf[64] = {0};
    int found_count = 0;
    float temp_val = 0.0f;
    memset(s_red_param_found, 0, sizeof(s_red_param_found));

    while (file.read_string(line_buf) == 0) {
        // 视觉仍用
        if (parse_line(line_buf, "small_err", &temp_val)) { Flash.small_err = (int)temp_val; found_count++; }
        else if (parse_line(line_buf, "picture_err", &temp_val)) { Flash.picture_err = (int)temp_val; found_count++; }
        else if (parse_line(line_buf, "debug_rgb_r_min", &temp_val)) { Flash.debug_rgb_r_min = (int)temp_val; found_count++; }
        else if (parse_line(line_buf, "debug_rgb_rb_diff", &temp_val)) { Flash.debug_rgb_rb_diff = (int)temp_val; found_count++; }
        else if (parse_line(line_buf, "debug_rgb_rg_diff", &temp_val)) { Flash.debug_rgb_rg_diff = (int)temp_val; found_count++; }
        // 遗留键：读入但不驱动控制
        else if (parse_line(line_buf, "kp", &temp_val)) { Flash.kp = temp_val; found_count++; }
        else if (parse_line(line_buf, "ki", &temp_val)) { Flash.ki = temp_val; found_count++; }
        else if (parse_line(line_buf, "kd", &temp_val)) { Flash.kd = temp_val; found_count++; }
        else if (parse_line(line_buf, "forward", &temp_val)) {
            Flash.forward = (int)temp_val;
            s_forward_base = temp_val;
            found_count++;
        }
        // 控制唯一键
        try_load(line_buf, "speed_kf_l", &s_speed_kf_l, &found_count);
        try_load(line_buf, "speed_kp_l", &s_speed_kp_l, &found_count);
        try_load(line_buf, "speed_ki_l", &s_speed_ki_l, &found_count);
        try_load(line_buf, "speed_kf_r", &s_speed_kf_r, &found_count);
        try_load(line_buf, "speed_kp_r", &s_speed_kp_r, &found_count);
        try_load(line_buf, "speed_ki_r", &s_speed_ki_r, &found_count);
        try_load(line_buf, "speed_max_i", &s_speed_max_i, &found_count);
        try_load(line_buf, "speed_max_out", &s_speed_max_out, &found_count);
        try_load(line_buf, "speed_max_error", &s_speed_max_error, &found_count);
        try_load(line_buf, "middle_kp", &s_middle_kp, &found_count);
        try_load(line_buf, "middle_kd", &s_middle_kd, &found_count);
        try_load(line_buf, "middle_ki", &s_middle_ki, &found_count);
        try_load(line_buf, "middle_max_out", &s_middle_max_out, &found_count);
        try_load(line_buf, "middle_ref_slew", &s_middle_ref_slew, &found_count);
        try_load(line_buf, "middle_tpm_weight", &s_middle_tpm_weight, &found_count);
        // 遗留键 middle_kff：曾用于错误的 FF+I-P 方案；忽略不应用
        try_load(line_buf, "image_kp_scale", &s_image_kp_scale, &found_count);
        try_load(line_buf, "image_kd", &s_image_kd, &found_count);
        try_load(line_buf, "threshold_static", &s_threshold_static, &found_count);
        try_load(line_buf, "camera_exposure", &s_camera_exposure, &found_count);
        try_load(line_buf, "base_speed", &s_base_speed, &found_count);
        try_load(line_buf, "straight_trim", &s_straight_trim, &found_count);
        try_load(line_buf, "picture_speed", &s_picture_speed, &found_count);
        try_load(line_buf, "ramp_speed", &s_ramp_speed, &found_count);
        try_load(line_buf, "picture_recog_k", &s_picture_recog_k, &found_count);
        try_load(line_buf, "picture_recog_base", &s_picture_recog_base, &found_count);
        try_load(line_buf, "picture_near_dist", &s_picture_near_dist, &found_count);
        try_load(line_buf, "picture_side_offset", &s_picture_side_offset, &found_count);
        try_load(line_buf, "picture_hold_dist", &s_picture_hold_dist, &found_count);
        try_load(line_buf, "picture_finish_dist", &s_picture_finish_dist, &found_count);
        try_load(line_buf, "picture_conf_min", &s_picture_conf_min, &found_count);
        try_load_red(line_buf, kRedBrickEnable, &s_red_brick_enable, &found_count);
        try_load_red(line_buf, kPictureRoiEnable, &s_picture_roi_enable, &found_count);
        try_load_red(line_buf, kRedRouteWhiteRatio, &s_red_route_white_ratio, &found_count);
        try_load_red(line_buf, kRedRouteConfirmFrames, &s_red_route_confirm_frames, &found_count);
        try_load_red(line_buf, kRedBrickSideDeadband, &s_red_brick_side_deadband, &found_count);
        try_load_red(line_buf, kRedBrickSlowSpeed, &s_red_brick_slow_speed, &found_count);
        try_load_red(line_buf, kRedBrickSpeed, &s_red_brick_speed, &found_count);
        try_load_red(line_buf, kRedBrickOffsetOut, &s_red_brick_offset_out, &found_count);
        try_load_red(line_buf, kRedBrickOffsetBack, &s_red_brick_offset_back, &found_count);
        try_load_red(line_buf, kRedBrickSlowdownMs, &s_red_brick_slowdown_ms, &found_count);
        try_load_red(line_buf, kRedBrickOutMs, &s_red_brick_out_ms, &found_count);
        try_load_red(line_buf, kRedBrickPastMs, &s_red_brick_past_ms, &found_count);
        try_load_red(line_buf, kRedBrickBackMs, &s_red_brick_back_ms, &found_count);
        try_load_red(line_buf, kRedBrickRecoverMs, &s_red_brick_recover_ms, &found_count);
        try_load(line_buf, "forward_base", &s_forward_base, &found_count);
        try_load(line_buf, "huandao_l_state5_exit_yaw", &s_huandao_l_state5_exit_yaw, &found_count);
        try_load(line_buf, "huandao_r_state5_exit_yaw", &s_huandao_r_state5_exit_yaw, &found_count);
        try_load(line_buf, "middle_test_target", &s_middle_test_target, &found_count);
        try_load(line_buf, "count_per_meter", &s_count_per_meter, &found_count);
        try_load(line_buf, "count_per_meter_l", &s_count_per_meter_l, &found_count);
        try_load(line_buf, "count_per_meter_r", &s_count_per_meter_r, &found_count);
        try_load(line_buf, "track_width_m", &s_track_width_m, &found_count);
        try_load(line_buf, "c_gyro", &s_c_gyro, &found_count);

        memset(line_buf, 0, sizeof(line_buf));
    }

    if (found_count > 0) {
        printf("[Param] Loaded %d key(s) from file.\r\n", found_count);
    } else {
        printf("[Param] File missing/empty. Using code defaults and writing template.\r\n");
        for (int i = 0; i < kRedParamCount; ++i)
            s_red_param_found[i] = 1;
        Param_SaveAll();
    }

    Param_ApplyToControllers();
}

void Param_SaveSingle(const char* key, float value) {
    char new_line[64] = {0};
    sprintf(new_line, "%s=%.4f\r\n", key, value);

    char full_content[4096] = {0};
    char line[64] = {0};
    int updated = 0;
    float temp_check = 0.0f;

    zf_driver_file_string f_in(PARAM_FILE_NAME, "r");
    while(f_in.read_string(line) == 0) {
        if (parse_line(line, key, &temp_check)) {
            strcat(full_content, new_line);
            updated = 1;
        } else {
            strcat(full_content, line);
            int len = strlen(full_content);
            if(len > 0 && full_content[len-1] != '\n') {
                strcat(full_content, "\r\n");
            }
        }
        memset(line, 0, sizeof(line));
    }
    if (!updated) {
        strcat(full_content, new_line);
    }
    zf_driver_file_string f_out(PARAM_FILE_NAME, "w");
    f_out.write_string(full_content);
    printf("[Param] Saved %s = %.4f\r\n", key, value);
}

void Param_SaveAll(void) {
    // 保存配置缓存 + 运行控制值；forward_base 用配置而非瞬时 forward1
    char content[4096] = {0};
    sprintf(content,
            "speed_kf_l=%.4f\r\n"
            "speed_kp_l=%.4f\r\n"
            "speed_ki_l=%.4f\r\n"
            "speed_kf_r=%.4f\r\n"
            "speed_kp_r=%.4f\r\n"
            "speed_ki_r=%.4f\r\n"
            "speed_max_i=%.4f\r\n"
            "speed_max_out=%.4f\r\n"
            "speed_max_error=%.4f\r\n"
            "middle_kp=%.4f\r\n"
            "middle_kd=%.4f\r\n"
            "middle_ki=%.4f\r\n"
            "middle_max_out=%.4f\r\n"
            "middle_ref_slew=%.4f\r\n"
            "middle_tpm_weight=%.4f\r\n"
            "image_kp_scale=%.4f\r\n"
            "image_kd=%.4f\r\n"
            "threshold_static=%.4f\r\n"
            "camera_exposure=%d\r\n"
            "base_speed=%.4f\r\n"
            "straight_trim=%.4f\r\n"
            "picture_speed=%.4f\r\n"
            "ramp_speed=%.4f\r\n"
            "picture_recog_k=%.4f\r\n"
            "picture_recog_base=%.4f\r\n"
            "picture_near_dist=%.4f\r\n"
            "picture_side_offset=%.4f\r\n"
            "picture_hold_dist=%.4f\r\n"
            "picture_finish_dist=%.4f\r\n"
            "picture_conf_min=%.4f\r\n"
            "red_brick_enable=%d\r\n"
            "picture_roi_enable=%d\r\n"
            "red_route_white_ratio=%.4f\r\n"
            "red_route_confirm_frames=%d\r\n"
            "red_brick_side_deadband=%d\r\n"
            "red_brick_slow_speed=%.4f\r\n"
            "red_brick_speed=%.4f\r\n"
            "red_brick_offset_out=%.4f\r\n"
            "red_brick_offset_back=%.4f\r\n"
            "red_brick_slowdown_ms=%u\r\n"
            "red_brick_out_ms=%u\r\n"
            "red_brick_past_ms=%u\r\n"
            "red_brick_back_ms=%u\r\n"
            "red_brick_recover_ms=%u\r\n"
            "forward_base=%.4f\r\n"
            "huandao_l_state5_exit_yaw=%.4f\r\n"
            "huandao_r_state5_exit_yaw=%.4f\r\n"
            "middle_test_target=%.4f\r\n"
            "count_per_meter=%.4f\r\n"
            "count_per_meter_l=%.4f\r\n"
            "count_per_meter_r=%.4f\r\n"
            "track_width_m=%.4f\r\n"
            "c_gyro=%.4f\r\n"
            "small_err=%d\r\n"
            "picture_err=%d\r\n"
            "debug_rgb_r_min=%d\r\n"
            "debug_rgb_rb_diff=%d\r\n"
            "debug_rgb_rg_diff=%d\r\n",
            Velocity_L.kf, Velocity_L.kp, Velocity_L.ki,
            Velocity_R.kf, Velocity_R.kp, Velocity_R.ki,
            Velocity_L.Max_integral, Velocity_L.Max_output, Velocity_L.Max_error,
            Dis_1.Kp, Dis_1.Kd, Dis_1.Ki, Dis_1.MAX_OutPut,
            g_middle_ref_slew_per_s, LPF_Speed,
            s_image_kp_scale, Image.Kd,
            (float)Threshold_static,
            Flash.camera_exposure,
            g_base_speed, g_straight_trim, g_picture_speed, g_ramp_speed,
            g_picture_recog_k, g_picture_recog_base, g_picture_near_dist,
            g_picture_side_offset, g_picture_hold_dist, g_picture_finish_dist,
            g_picture_conf_min,
            s_applied_red_router_config.brick_enable ? 1 : 0,
            s_applied_red_router_config.picture_roi_enable ? 1 : 0,
            s_applied_red_router_config.white_ratio,
            (int)s_applied_red_router_config.confirm_frames,
            (int)s_applied_red_router_config.side_deadband,
            s_applied_red_detour_config.slow_speed,
            s_applied_red_detour_config.past_speed,
            s_applied_red_detour_config.offset_out,
            s_applied_red_detour_config.offset_back,
            (unsigned)s_applied_red_detour_config.slowdown_ms,
            (unsigned)s_applied_red_detour_config.out_ms,
            (unsigned)s_applied_red_detour_config.past_ms,
            (unsigned)s_applied_red_detour_config.back_ms,
            (unsigned)s_applied_red_detour_config.recover_ms,
            s_forward_base, g_huandao_l_state5_exit_yaw, g_huandao_r_state5_exit_yaw,
            g_middle_test_target,
            g_count_per_meter, g_count_per_meter_l, g_count_per_meter_r,
            g_track_width_m, g_c_gyro,
            Flash.small_err, Flash.picture_err,
            Flash.debug_rgb_r_min, Flash.debug_rgb_rb_diff, Flash.debug_rgb_rg_diff);

    zf_driver_file_string file(PARAM_FILE_NAME, "w");
    file.write_string(content);
    printf("[Param] All params saved to %s\r\n", PARAM_FILE_NAME);
}
