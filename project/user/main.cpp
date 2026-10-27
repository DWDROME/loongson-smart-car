#include "zf_common_headfile.hpp"
#include "detour/red_brick_detour_runtime.hpp"
#include <cstdlib>
#include <cstring>

// =============================================================================
// main 组织方式对照 Car_Round/user/main.cpp：
//   1) 读环境/命令行：RUN_MODE + OUTPUT_MODE
//   2) 硬件 init → PID 默认 → 参数文件 → 控制清零
//   3) 按输出互斥初始化 VOFA 或 AssistantV2（失败不杀进程）
//   4) 图像 init
//   5) 最后开 5ms pit
//   6) 主循环：图像处理 + 上位机发送（不在 pit 里发 TCP）
// 运行级按键不在本进程：见 tools/key_supervisor（KEY1/2/4）
// =============================================================================

uint8 Tcp_buffer[128];
int frame_count = 0;
auto start_time = std::chrono::steady_clock::now();
auto last_print_time = start_time;
#define PRINT_INTERVAL 1000

int32_t frame_cnt = 0;
static uint32_t s_assistant_frame_id = 0;

int8_t g_run_mode = RUN_MODE_STOP;
int8_t g_output_mode = OUTPUT_MODE_NONE;
// 默认 0：mode1 走速度环闭环（base<=0 时 target=0 零速 PI，可调 Ki）
// 仅手推 cpm 显式 --freewheel / SPEED_FREEWHEEL=1
int8_t g_speed_freewheel = 0;
// 安全默认：未标定前基础速度必须为 0（TRACK 硬停；mode1 零速闭环≠松电机）
float g_base_speed = 0.0f;
float g_straight_trim = 0.025f;
// ae3379a 作者预设；由 params_config.txt 覆盖，不是实时规划结果。
float g_picture_speed = 100.0f;
float g_ramp_speed = 150.0f;
float g_picture_recog_k = 0.1f;
float g_picture_recog_base = 40.0f;
float g_picture_near_dist = 30.0f;
float g_picture_side_offset = 10.0f;
float g_picture_hold_dist = 30.0f;
float g_picture_finish_dist = 40.0f;
float g_picture_conf_min = 0.50f;
float g_middle_test_target = 0.0f;
float g_image_kp_scale = 1.0f;
uint32_t g_control_tick = 0;
float g_control_dt_ms = 5.0f;

static auto s_last_control_tp = std::chrono::steady_clock::now();
static red_event::RedBrickDetour s_track_brick_detour;
static red_event::BrickDetourCommand s_track_brick_command;
static uint32_t s_track_brick_event_id = 0;

static uint64_t monotonic_ms(const std::chrono::steady_clock::time_point &now)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count());
}

static void publish_track_brick_state(void)
{
    red_event::BrickDetourRuntimeSnapshot snapshot;
    snapshot.event_id = s_track_brick_event_id;
    snapshot.state = s_track_brick_detour.state();
    snapshot.direction = s_track_brick_detour.direction();
    red_event::PublishBrickDetourState(snapshot);
}

static void cancel_track_brick_detour(const char *reason)
{
    if (s_track_brick_detour.state() != red_event::BrickDetourState::kIdle)
    {
        printf("[BRICK_DETOUR_TRACK] cancel event=%u state=%s reason=%s\n",
               s_track_brick_event_id,
               red_event::ToString(s_track_brick_detour.state()), reason);
    }
    s_track_brick_detour.Cancel();
    s_track_brick_command = {};
    s_track_brick_event_id = 0;
    red_event::ResetBrickDetourRuntime();
}

static red_event::BrickDetourCommand update_track_brick_detour(
    const std::chrono::steady_clock::time_point &now)
{
    const red_event::BrickDetourState previous_state = s_track_brick_detour.state();
    red_event::BrickDetourRequest request;
    if (red_event::ConsumeBrickDetourRequest(request))
    {
        const bool element_clear =
            Flag.picture == 0 && Flag.Huandao_L == 0 && Flag.Huandao_R == 0 &&
            Flag.ramp == 0 && Flag.Zebra_cross == 0 && Flag.small_rock == 0;
        if (!element_clear)
        {
            printf("[BRICK_DETOUR_TRACK] reject event=%u reason=element-conflict "
                   "pic=%u HL=%u HR=%u ramp=%u zebra=%u rock=%u\n",
                   request.event_id, (unsigned)Flag.picture,
                   (unsigned)Flag.Huandao_L, (unsigned)Flag.Huandao_R,
                   (unsigned)Flag.ramp, (unsigned)Flag.Zebra_cross,
                   (unsigned)Flag.small_rock);
        }
        else if (s_track_brick_detour.Start(request.direction, monotonic_ms(now)))
        {
            s_track_brick_event_id = request.event_id;
            printf("[BRICK_DETOUR_TRACK] start event=%u avoid=%s\n",
                   request.event_id, red_event::ToString(request.direction));
        }
        else
        {
            printf("[BRICK_DETOUR_TRACK] reject event=%u reason=fsm-busy state=%s\n",
                   request.event_id,
                   red_event::ToString(s_track_brick_detour.state()));
        }
    }

    s_track_brick_command = s_track_brick_detour.Update(monotonic_ms(now));
    publish_track_brick_state();
    if (previous_state != s_track_brick_detour.state())
    {
        if (s_track_brick_detour.state() == red_event::BrickDetourState::kPast)
        {
            // Incremental middle loop retains output; PAST must not inherit OUT steering.
            Dis_1.OutPut = 0.0f;
            Dis_1.Error = 0.0f;
            Dis_1.Last_Error = 0.0f;
            Dis_1.Last_Last_Error = 0.0f;
            Dis_1.Integral = 0.0f;
            Middle_ReferenceReset();
        }
        printf("[BRICK_DETOUR_TRACK] event=%u state=%s active=%u ignore_image=%u "
               "speed_valid=%u speed=%.1f provider_offset=%.1f\n",
               s_track_brick_event_id,
               red_event::ToString(s_track_brick_detour.state()),
               s_track_brick_command.active ? 1u : 0u,
               s_track_brick_command.ignore_image_steering ? 1u : 0u,
               s_track_brick_command.speed_override_valid ? 1u : 0u,
               s_track_brick_command.speed_goal,
               s_track_brick_command.dir_offset);
    }
    return s_track_brick_command;
}

static void motors_stop_and_clear(void)
{
    Control_State_Reset();
    set_pwm(0, 0);
}

static void run_speed_pair(void)
{
    // 审查：target/feedback 全程 float，禁止 int16 截断吃掉亚 count 反馈
    const float out_r = Speed_PID_Cal(&Velocity_R, v_right_target, encoder_R.speed);
    const float out_l = Speed_PID_Cal(&Velocity_L, v_left_target, encoder_L.speed);
    PWM_R = (int16_t)out_r;
    PWM_L = (int16_t)out_l;
    set_pwm(PWM_L, PWM_R);
}

static void allocate_diff_targets(void)
{
    // ae3379a 原始差速：K_turn=2 单边承担全部差速（内轮多降）。
    // speed_goal=0 时会单边倒车；零速拧头属该结构副作用，不在此改拓扑。
    const float K_turn = 2.0f;
    if (Dis_Out > 0)
    {
        v_right_target = speed_goal - Dis_Out * K_turn;
        v_left_target = speed_goal + Dis_Out * (2 - K_turn);
    }
    else
    {
        v_right_target = speed_goal - Dis_Out * (2 - K_turn);
        v_left_target = speed_goal + Dis_Out * K_turn;
    }
}

// ---------- 5ms 控制（pit 只做实时控制，不发网） ----------
void pit_callback()
{
    auto now = std::chrono::steady_clock::now();
    g_control_dt_ms = (float)std::chrono::duration<double, std::milli>(now - s_last_control_tp).count();
    if (g_control_dt_ms <= 0.0f)
        g_control_dt_ms = 5.0f;
    s_last_control_tp = now;
    g_control_tick++;

    ICM_getEulerianAngles();
    if (it_time % 10 == 0)
        dl1x_distance_raw = dl1x_dev.get_distance();
    Encoder_update();
    distance_judge();

    // 元素紧急停车
    if (run_flag == 2)
    {
        cancel_track_brick_detour("emergency-stop");
        motors_stop_and_clear();
        if (g_output_mode == OUTPUT_MODE_VOFA)
            Vofa_FillSnapshot();
        it_time++;
        return;
    }

    switch (g_run_mode)
    {
    case RUN_MODE_TRACK:
    {
        // 合同：base_speed<=0 硬停（方案 C：外环只在有前进速度时标定，不做零速原地）
        if (g_base_speed <= 0.0f)
        {
            cancel_track_brick_detour("base-speed-zero");
            run_flag = 0;
            motors_stop_and_clear();
            break;
        }
        // 相机未就绪或图像过期：硬停，禁止沿用旧 Image_out 转向
        g_image_age_ms = Image_AgeMs_Now();
        if (!g_camera_ok || g_image_age_ms > IMAGE_STALE_MS)
        {
            cancel_track_brick_detour("camera-stale");
            run_flag = 0;
            motors_stop_and_clear();
            break;
        }
        run_flag = 1;
        const red_event::BrickDetourCommand brick_command =
            update_track_brick_detour(now);
        // 前瞻：作者式随速后移；基准行由参数 forward_base 给出（默认 30）
        forward1 = (int)g_forward_base - (int)(Master_Speed / 60);
        if (forward1 < 1)
            forward1 = 1;
        // 速度：直道 base；picture/ramp 默认保留 ae3379a 预设，可由参数文件覆盖。
        speed_goal = g_base_speed;
        if (Flag.picture == 2)
            speed_goal = g_picture_speed;
        if (Flag.ramp == 2)
            speed_goal = g_ramp_speed;
        if (brick_command.speed_override_valid)
            speed_goal = brick_command.speed_goal;
        // 中环增量 PI：r=-Image_out；feedback=Dis_Speed（G/Tpm 权重由参数加载）
        // Dout 有记忆；slew=0；±MAX 安全边界；Kd=0
        {
            const float d_ref_raw = brick_command.ignore_image_steering ? 0.0f : -Image_out;
            const float middle_reference =
                Middle_ReferenceUpdate(d_ref_raw, g_control_dt_ms);
            Dis_Out = Dis_PID_Calculate(&Dis_1, middle_reference, Dis_Speed);
        }
        // 直线机械偏置与转向输出共用同一安全边界，避免 trim 绕过中环限幅。
        Dis_Out += speed_goal * g_straight_trim;
        Dis_Out = LIMIT(Dis_Out, -Dis_1.MAX_OutPut, Dis_1.MAX_OutPut);
        // K_turn=2: red-target approach and detour must never reverse an inner wheel.
        // Normal lane tracking remains unchanged outside picture=2/detour.
        Dis_Out = red_event::LimitBrickDetourDifferential(
            speed_goal, Dis_Out, brick_command.active || Flag.picture == 2);
        allocate_diff_targets();
        run_speed_pair();
        break;
    }

    case RUN_MODE_SPEED_TEST:
        // SPEED_TEST：纯速度直线；target=max(base,0)。
        // base<=0 → 零速 PI（motors armed）；手推 cpm 仅 freewheel。
        // 中后段左偏根因：闭环把左右 count 拉齐后，地速仍可能不等（cpm/机械），
        // 仅改 Kf 无法纠正稳态偏航。目标按 cpm 归一 + 直线 trim（>0 抬左压右）。
        Image_out = 0;
        Dis_Out = 0;
        if (g_speed_freewheel)
        {
            run_flag = 0;
            speed_goal = 0;
            v_left_target = 0;
            v_right_target = 0;
            motors_stop_and_clear(); // 松电机：不闭环、每拍清积分
        }
        else
        {
            run_flag = 1;
            speed_goal = (g_base_speed > 0.0f) ? g_base_speed : 0.0f;
            // 等速地速：count 目标 ∝ cpm；再叠加 trim 纠换板后残余左偏
            const float cpm_l = (g_count_per_meter_l > 1.0f) ? g_count_per_meter_l : g_count_per_meter;
            const float cpm_r = (g_count_per_meter_r > 1.0f) ? g_count_per_meter_r : g_count_per_meter;
            const float cpm_m = 0.5f * (cpm_l + cpm_r);
            // >0：左目标↑ 右目标↓ → 左快右慢 → 车往右拧（纠左偏）
            // <0：抬右压左 → 纠右偏
            if (speed_goal > 0.0f && cpm_m > 1.0f)
            {
                v_left_target = speed_goal * (cpm_l / cpm_m) * (1.0f + g_straight_trim);
                v_right_target = speed_goal * (cpm_r / cpm_m) * (1.0f - g_straight_trim);
            }
            else
            {
                v_left_target = speed_goal;
                v_right_target = speed_goal;
            }
            run_speed_pair();
        }
        break;

    case RUN_MODE_MIDDLE_TEST:
        // 允许 base_speed=0：零速 + 中环（拧车头看 G_dis 纠正）。外环恒旁路。
        // 与 TRACK 的 base<=0 硬停不同；与 SPEED_TEST 默认零速闭环同属显式标定语义。
        run_flag = 1;
        speed_goal = (g_base_speed > 0.0f) ? g_base_speed : 0.0f;
        Image_out = 0;
        Dis_Out = Dis_PID_Calculate(&Dis_1, g_middle_test_target, Dis_Speed);
        allocate_diff_targets();
        run_speed_pair();
        break;

    case RUN_MODE_SENSOR_CAL:
    case RUN_MODE_IMAGE_TEST:
    case RUN_MODE_CAMERA_FOCUS:
        // 锁电机：每拍主动 0（Car_Round menu=30 同类）
        cancel_track_brick_detour("motors-locked-mode");
        run_flag = 0;
        motors_stop_and_clear();
        break;

    case RUN_MODE_STOP:
    default:
        cancel_track_brick_detour("stop-mode");
        run_flag = 0;
        motors_stop_and_clear();
        break;
    }

    if (g_output_mode == OUTPUT_MODE_VOFA)
        Vofa_FillSnapshot();

    // 圆环状态变化 + 状态 6 全程 + 出环后三图像帧：只观测交接连续性。
    // 不改变圆环补线、退出条件或任何控制量。
    {
        static uint8_t s_prev_hl = 0;
        static uint8_t s_prev_hr = 0;
        static int s_ring_diag_left = 0;
        static int s_ring_diag_right = 0;
        static uint32_t s_last_diag_frame = 0;

        const bool l_exit_window = (g_huandao_L_geom == 6) ||
                                   (s_prev_hl == 6 && Flag.Huandao_L == 0);
        const bool r_exit_window = (g_huandao_R_geom == 6) ||
                                   (s_prev_hr == 6 && Flag.Huandao_R == 0);

        if (Flag.Huandao_L != s_prev_hl)
        {
            s_ring_diag_left = 3; // 变化帧 + 后两帧
            s_prev_hl = Flag.Huandao_L;
        }
        if (Flag.Huandao_R != s_prev_hr)
        {
            s_ring_diag_right = 3;
            s_prev_hr = Flag.Huandao_R;
        }
        // 旧日志只覆盖状态变化附近三帧，抓不到状态 6 内部哪一帧翻向。
        // 每个新图像帧都保留状态 6；进入 0 后再保留三帧观察交接结果。
        if (l_exit_window)
            s_ring_diag_left = 3;
        if (r_exit_window)
            s_ring_diag_right = 3;

        if ((s_ring_diag_left > 0 || s_ring_diag_right > 0) &&
            g_image_frame_id != s_last_diag_frame)
        {
            s_last_diag_frame = g_image_frame_id;
            int fr = forward;
            if (fr < 0)
                fr = 0;
            if (fr >= LCDH_0)
                fr = LCDH_0 - 1;
            const float L = (float)Left_Sideline[fr];
            const float R = (float)Right_Sideline[fr];
            const float Mid = 0.5f * (L + R);
            printf("RING HLg:%u HL:%u HRg:%u HR:%u Yerr:%.1f fwd:%d "
                   "L:%.1f R:%.1f Lf:%u Rf:%u Ls:%u Rs:%u dist:%.1f "
                   "Ll:%d Rl:%d C:%u%u%u%u Mid:%.1f "
                   "Dir:%.1f Img:%.2f r:%.2f G:%.2f Tpm:%.2f "
                   "dP:%.2f dI:%.2f dU:%.2f Dout:%.2f\n",
                   (unsigned)g_huandao_L_geom, (unsigned)Flag.Huandao_L,
                   (unsigned)g_huandao_R_geom, (unsigned)Flag.Huandao_R,
                   Yaw_Huandao_err, fr,
                   L, R, (unsigned)Left_Sideline_flag[fr],
                   (unsigned)Right_Sideline_flag[fr],
                   (unsigned)imgInfo.L_straight_flag,
                   (unsigned)imgInfo.R_straight_flag, distance,
                   imgInfo.L_loselineSum, imgInfo.R_loselineSum,
                   (unsigned)L_h_guai.flag, (unsigned)L_l_guai.flag,
                   (unsigned)R_h_guai.flag, (unsigned)R_l_guai.flag, Mid,
                   Dir_err, Image_out, g_middle_reference, G_dis, Tpm_Dis,
                   g_middle_delta_p, g_middle_delta_i, g_middle_delta, Dis_Out);
            if (s_ring_diag_left > 0)
                s_ring_diag_left--;
            if (s_ring_diag_right > 0)
                s_ring_diag_right--;
        }
    }

    it_time++;
    // 5ms pit：默认每 20 拍=100ms；mode1/3 调参每 2 拍=10ms
    const int print_div =
        (g_run_mode == RUN_MODE_SPEED_TEST || g_run_mode == RUN_MODE_MIDDLE_TEST) ? 2
                                                                                 : 20;
    if (it_time % print_div == 0)
    {
        // r/mref/G/Tpm/e/dP/dI/dU/Dout；slew=0 时 rawref==mref
        // HLg/HRg=本帧几何入口；HL/HR=逻辑下一状态
        // eL/eR=编码器实测转速；vL/vR=目标。速度环标定必须看 e 而非 v。
        printf("mode:%d Yaw:%.3f HLg:%u HL:%u HRg:%u HR:%u Yerr:%.1f fwd:%d top:%d "
               "Dir:%.1f Img:%.2f r:%.2f mref:%.2f G:%.2f Tpm:%.2f e:%.2f "
               "dP:%.2f dI:%.2f dU:%.2f Dout:%.2f "
               "vL:%.1f vR:%.1f eL:%.1f eR:%.1f V:%.1f goal:%.0f Cfit:%.2f n:%u "
               "pic:%u Red:%u inf:%u W:%u S:%u Vh:%u redn:%zu "
               "brick:%u bevt:%u bact:%u bign:%u boff:%.1f "
               "Lh:%u Rh:%u Ll:%u Rl:%u\n",
               (int)g_run_mode, icm_data.yaw,
               (unsigned)g_huandao_L_geom, (unsigned)Flag.Huandao_L,
               (unsigned)g_huandao_R_geom, (unsigned)Flag.Huandao_R,
               Yaw_Huandao_err, forward, imgInfo.top,
               Dir_err, Image_out, g_middle_ref_raw, g_middle_reference, G_dis,
               Tpm_Dis, Dis_1.Error, g_middle_delta_p, g_middle_delta_i,
               g_middle_delta, Dis_Out,
               v_left_target, v_right_target,
               encoder_L.speed, encoder_R.speed,
               Master_Speed, speed_goal,
               g_cgyro_fit, (unsigned)g_cgyro_fit_n,
               (unsigned)Flag.picture, (unsigned)Flag.Redblock, (unsigned)Flag.infer,
               (unsigned)Flag.weapon, (unsigned)Flag.supply, (unsigned)Flag.vehicle,
               red_objects.size(),
               (unsigned)s_track_brick_detour.state(), s_track_brick_event_id,
               s_track_brick_command.active ? 1u : 0u,
               s_track_brick_command.ignore_image_steering ? 1u : 0u,
               s_track_brick_command.dir_offset,
               (unsigned)L_h_guai.flag, (unsigned)R_h_guai.flag,
               (unsigned)L_l_guai.flag, (unsigned)R_l_guai.flag);
    }
}

// ---------- 启动参数（对齐 Car_Round 的 MENU_FLAG / OUTPUT_FLAG 环境变量） ----------
static void parse_cli(int argc, char **argv)
{
    const char *env_mode = getenv("RUN_MODE");
    const char *env_out = getenv("OUTPUT_MODE");
    const char *env_fw = getenv("SPEED_FREEWHEEL");
    if (env_mode && IsKnownRunMode(atoi(env_mode)))
        g_run_mode = (int8_t)atoi(env_mode);
    if (env_out)
    {
        if (strcmp(env_out, "vofa") == 0)
            g_output_mode = OUTPUT_MODE_VOFA;
        else if (strcmp(env_out, "assistant") == 0)
            g_output_mode = OUTPUT_MODE_ASSISTANT;
        else
            g_output_mode = OUTPUT_MODE_NONE;
    }
    // 手推 cpm：环境或 CLI 显式打开；base_speed=0 不再表示松电机
    if (env_fw && env_fw[0] != '\0' && env_fw[0] != '0')
        g_speed_freewheel = 1;

    const char *host_ip = SERVER_IP;
    const int host_port = PORT;
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--vofa") == 0)
            g_output_mode = OUTPUT_MODE_VOFA;
        else if (strcmp(argv[i], "--assistant") == 0)
            g_output_mode = OUTPUT_MODE_ASSISTANT;
        else if (strcmp(argv[i], "--none") == 0)
            g_output_mode = OUTPUT_MODE_NONE;
        else if (strcmp(argv[i], "--freewheel") == 0)
            g_speed_freewheel = 1;
        else if ((strcmp(argv[i], "--vofa-ip") == 0 ||
                  strcmp(argv[i], "--vofa-port") == 0) && i + 1 < argc)
        {
            printf("[main] ignore fixed-upper override: %s %s\n", argv[i], argv[i + 1]);
            ++i;
        }
        else if (IsKnownRunMode(atoi(argv[i])))
            g_run_mode = (int8_t)atoi(argv[i]);
        else
            printf("[main] ignore arg: %s\n", argv[i]);
    }

    printf("[main] run_mode=%d output_mode=%d freewheel=%d host=%s:%d\n",
           (int)g_run_mode, (int)g_output_mode, (int)g_speed_freewheel, host_ip, host_port);

    if (g_output_mode == OUTPUT_MODE_VOFA)
    {
        Vofa_InitTcp(host_ip, host_port);
    }
    else if (g_output_mode == OUTPUT_MODE_ASSISTANT)
    {
        // 失败不 return -1：对照任务 R9，控制继续（与 Car_Round 略不同：Car 会退出）
        if (assistant_v2_init() < 0)
            printf("[main] assistant init failed, continue without image uplink\n");
    }
}

int main(int argc, char **argv)
{
    setvbuf(stdout, nullptr, _IOLBF, 0);
    setvbuf(stderr, nullptr, _IOLBF, 0);

    // ---- 与 Car_Round 相同分层：先环境与模式，再硬件，再输出通道，再相机，最后定时器 ----
    init();
    pid_init();
    Param_Init();
    s_track_brick_detour.Configure(Param_RedBrickDetourConfig());
    parse_cli(argc, argv);
    if (!Param_IsApplyOk() &&
        (g_run_mode == RUN_MODE_TRACK ||
         g_run_mode == RUN_MODE_SPEED_TEST ||
         g_run_mode == RUN_MODE_MIDDLE_TEST))
    {
        printf("[main] parameter validation failed; force motor-capable mode %d -> STOP\n",
               (int)g_run_mode);
        g_run_mode = RUN_MODE_STOP;
    }
    Control_State_Reset();
    red_event::ResetBrickDetourRuntime();
    set_pwm(0, 0);

    // 全模式统一 G_dis 零偏：TRACK / 速度环 / 中环启动前静置（手必须不动；无死区）
    if (g_run_mode == RUN_MODE_TRACK ||
        g_run_mode == RUN_MODE_SPEED_TEST ||
        g_run_mode == RUN_MODE_MIDDLE_TEST)
    {
        Middle_GyroStillRecalibrate();
        Control_State_Reset();
        set_pwm(0, 0);
    }
    if (image_init() != 0)
        printf("[main] vision init failed: TRACK will hard-stop until model and camera are ok\n");
    else
        printf("[main] camera ok (TRACK requires fresh frames, age<%.0fms)\n", IMAGE_STALE_MS);

    // 最后启动 5ms：避免 init 阶段误驱动
    pit_timer.init_ms(5, pit_callback);
    printf("[main] pit 5ms started; keys via key_supervisor (not this process)\n");

    // cpm 累计：mode1 均清零。手推仅 freewheel；默认闭环（含 base=0 零速 PI）。
    if (g_run_mode == RUN_MODE_SPEED_TEST)
    {
        system_delay_ms(20);
        Cpm1m_ResetTotals();
        if (g_speed_freewheel)
        {
            printf("[cpm1m] FREEWHEEL mode1 motors free/PWM0 (not closed-loop), totals cleared.\r\n"
                   "[cpm1m] Align start → push straight 0.3m or 1.0m → say stop.\r\n"
                   "[cpm1m] cpm_l=sumL/dist_m  cpm_r=sumR/dist_m\r\n");
        }
        else if (g_base_speed <= 0.0f)
        {
            printf("[speed] ZERO-HOLD closed-loop mode1 base=0 motors ARMED (FF+PI, Ki effective).\r\n"
                   "[speed] Raise speed_ki and restart to see Integral/i_term on VOFA.\r\n");
        }
        else
        {
            printf("[speed] MOTOR-DRIVE mode1 base=%.0f closed-loop (not freewheel).\r\n",
                   g_base_speed);
        }
    }

    while (1)
    {
        // [测试关闭] 元素误检会常开蜂鸣，像报警。元素标定前保持关闭；需要时恢复下方分支。
        // if (Flag.picture > 0 || Flag.small_rock != 0 || Flag.ramp != 0 ||
        //     Flag.Zebra_cross == 1 || Flag.Zebra_cross == 4)
        //     beep.set_level(1);
        // else
        //     beep.set_level(0);
        beep.set_level(0);

        frame_count++;
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> elapsed = current_time - last_print_time;
        if (elapsed.count() >= PRINT_INTERVAL)
        {
            double fps = frame_count / (elapsed.count() / 1000.0);
            if (g_run_mode == RUN_MODE_SPEED_TEST)
            {
                const double sL = (double)encode_l_total;
                const double sR = (double)encode_r_total;
                printf("[cpm1m] sumL=%d sumR=%d | /0.3m → L=%.1f R=%.1f | /1.0m → L=%.1f R=%.1f | "
                       "inst=%d/%d V=%.1f base=%.0f\n",
                       (int)encode_l_total, (int)encode_r_total,
                       sL / 0.3, sR / 0.3, sL, sR,
                       (int)encoder_L.count_now, (int)encoder_R.count_now,
                       Master_Speed, g_base_speed);
            }
            else
            {
                printf("encoder_L:%d encoder_R:%d mode:%d\n",
                       encoder_L.count_now, encoder_R.count_now, (int)g_run_mode);
            }
            std::cout << "实时帧率：" << fps << " FPS | 累计帧数：" << frame_count << std::endl;
            frame_count = 0;
            last_print_time = current_time;
        }

        // 仅锁电机/停车时扫显示键；运行级 KEY1/2/4 不在此
        if (RunModeMotorsLocked(g_run_mode) || g_run_mode == RUN_MODE_STOP)
        {
            oled_show();
            key_scan();
        }

        ImageDeal();

        // 上位机：ImageDeal 同帧后发送（对齐 Car_Round main 末尾 assistant_v2_tick / vofa_send）
        if (g_output_mode == OUTPUT_MODE_ASSISTANT)
        {
            s_assistant_frame_id++;
            assistant_v2_tick(s_assistant_frame_id);
        }
        Vofa_TrySendFromMain();
    }
}
