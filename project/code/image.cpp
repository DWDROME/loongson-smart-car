#include "zf_common_headfile.hpp"
#include "dataset_capture.hpp"
#include "lq_ncnn.hpp"
#include "race_recognition_result.hpp"
#include "detour/red_brick_detour.hpp"
#include "detour/red_brick_detour_runtime.hpp"
#include "detour/red_brick_filter.hpp"
#include "detour/red_event_router.hpp"
#include "ww_transmission.h"
#include "run_mode.h"
#include <chrono>
#include <cerrno>
#include <cstring>
#include <sys/stat.h>
#include <unistd.h>
// 大佬明确说明：原图裁 80x80（包含红色区域），再缩放为 64x64 模型输入。
// marker=(40,63) 由数据集红标约 (32,50) 按 80->64 像素中心映射推导，待实机落盘复核。
#define CLASSIFIER_SOURCE_ROI_SIZE 80
#define CLASSIFIER_SOURCE_MARKER_X 40
#define CLASSIFIER_SOURCE_MARKER_Y 63
// Board/Brick density uses the same red anchor but a direct physical 64x64 crop.
#define DENSITY_ROI_SIZE 64
#define DENSITY_MARKER_X 32
#define DENSITY_MARKER_Y 50
// 下列 ROI 只用于旧的局部图像辅助逻辑，不参与 NCNN 分类。
#define REC_ROI_WIDTH   60
#define REC_ROI_HEIGHT  60
// ROI 微调：默认 0 对齐训练；若板上构图偏可改（改后需与产 ROI 一致）
#define ROI_SHIFT_X     0
#define ROI_SHIFT_Y     0
// 决策门限由唯一参数 g_picture_conf_min 提供，默认保持作者链路使用的 0.50。
// 红色候选先按直接物理 ROI64 判 Board/Brick；Board 分支随后再要求 ROI80 合法。
#define DENSITY_CENTER_X_MIN (DENSITY_MARKER_X - ROI_SHIFT_X)
#define DENSITY_CENTER_X_MAX (320 - (DENSITY_ROI_SIZE - DENSITY_MARKER_X) - ROI_SHIFT_X)
#define DENSITY_CENTER_Y_MIN (DENSITY_MARKER_Y - ROI_SHIFT_Y)
#define DENSITY_CENTER_Y_MAX (240 - (DENSITY_ROI_SIZE - DENSITY_MARKER_Y) - ROI_SHIFT_Y)
#define RED_CENTER_X_MIN (CLASSIFIER_SOURCE_MARKER_X - ROI_SHIFT_X)
#define RED_CENTER_X_MAX (320 - (CLASSIFIER_SOURCE_ROI_SIZE - CLASSIFIER_SOURCE_MARKER_X) - ROI_SHIFT_X)
#define RED_CENTER_Y_MIN (CLASSIFIER_SOURCE_MARKER_Y - ROI_SHIFT_Y)
#define RED_CENTER_Y_MAX (240 - (CLASSIFIER_SOURCE_ROI_SIZE - CLASSIFIER_SOURCE_MARKER_Y) - ROI_SHIFT_Y)
// 连通域最小面积：滤左缘噪点（训练侧 min_marker 也有过滤）
#define RED_MIN_AREA_KEEP 8
zf_device_uvc uvc_dev;//初始化摄像头对象
uint8_t Image_Zip[LCDH_1][LCDW_1] = {0}; //压缩后的图像数组
uint8_t Image_Use[LCDH_1][LCDW_1] = {0};//(经过大津法，膨胀，腐蚀后的)二值化的图像
cv::VideoCapture cap;
unsigned char Image_IFS[LCDH_1][LCDW_1];
// 4m 单帧红标链路使用大佬 64x64/ImageNet 三分类 NCNN 后端。
LQ_NCNN classifier;
uint8_t Threshold = 0;  //大津法求出的阈值
struct imageInformation imgInfo;
struct YuanSu Flag;
cv::Mat frame, grayFrame, binaryFrame,resizedFrame,flippedFrame,translatedFrame,translationMatrix,float_img,resized_img;
TransmissionStreamServer camera_server;
int roi_x_1= 0;
int roi_y_1 = 0;
int ROI_SIZE = CLASSIFIER_SOURCE_ROI_SIZE;
char txt[80];
bool zf_init_flag = false;
bool first_frame_logged = false;
#define ENABLE_FIRST_FRAME_DEBUG     true
int picture_cnt = 0;
bool first_flag = true;
int roix1,roiy1 = 0;
using namespace cv;
cv::Mat lq_frame;

namespace
{
constexpr int kImageTestCaptureLimit = 10;
constexpr int kImageTestCaptureInferInterval = 1;
constexpr double kFocusPublishPeriodMs = 200.0;
constexpr int kFocusStatusHeight = 80;
constexpr int kFocusSidePanelWidth = 80;
int s_image_test_capture_count = 0;
int s_image_test_infer_count = 0;
bool s_focus_result_valid = false;
int s_focus_class_idx = -1;
float s_focus_confidence = 0.0f;
float s_focus_distance_cm = 0.0f;
bool s_focus_roi_valid = false;
cv::Rect s_focus_roi_rect;
cv::Point s_focus_red_center;
cv::Mat s_focus_model_input;
bool s_focus_direct64_valid = false;
cv::Rect s_focus_direct64_rect;
cv::Mat s_focus_direct64_input;
bool s_focus_direct64_result_valid = false;
int s_focus_direct64_class_idx = -1;
float s_focus_direct64_confidence = 0.0f;
red_event::RedEventRouter s_red_event_router;
red_event::RedBrickDetour s_red_brick_detour;
red_event::RedRouteResult s_red_route_result;
red_event::BrickDetourCommand s_red_detour_command;
int s_red_signed_side_error = 0;
bool s_red_route_observed_this_frame = false;

std::uint64_t MonotonicMs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool IsRedRouteDiagnosticMode()
{
    return g_run_mode == RUN_MODE_IMAGE_TEST || g_run_mode == RUN_MODE_CAMERA_FOCUS;
}

bool IsRedRouteEnabledMode()
{
    return g_run_mode == RUN_MODE_TRACK || IsRedRouteDiagnosticMode();
}

bool IsRedTargetProcessingEnabled()
{
    const red_event::RedEventRouterConfig &config = s_red_event_router.config();
    return config.brick_enable || config.picture_roi_enable;
}

bool IsBrickLocked(red_event::RedEventKind kind)
{
    return kind == red_event::RedEventKind::kBrickLeftLocked ||
           kind == red_event::RedEventKind::kBrickRightLocked;
}

bool IsBoardLocked(red_event::RedEventKind kind)
{
    return kind == red_event::RedEventKind::kBoardLocked;
}

void PublishRedRouteResult(const red_event::RedRouteResult &result,
                           bool white_ratio_valid, float white_ratio,
                           int signed_side_error)
{
    const red_event::RedEventKind previous_kind = s_red_route_result.kind;
    const std::uint8_t previous_count = s_red_route_result.confirm_count;
    s_red_route_result = result;
    s_red_signed_side_error = signed_side_error;
    if (previous_kind != result.kind || previous_count != result.confirm_count || result.new_event)
    {
        printf("[RED_ROUTE] mode=%d frame=%u kind=%s white_ratio=%.4f valid=%u side_err=%d "
               "confirm=%u/%u locked=%u new=%u event=%u board_ncnn=%u "
               "brick_side=%s plan_avoid=%s\n",
               static_cast<int>(g_run_mode),
               g_image_frame_id + 1,
               red_event::ToString(result.kind),
               white_ratio_valid ? white_ratio : -1.0f,
               white_ratio_valid ? 1u : 0u,
               signed_side_error,
               static_cast<unsigned>(result.confirm_count),
               static_cast<unsigned>(s_red_event_router.config().confirm_frames),
               result.locked ? 1u : 0u, result.new_event ? 1u : 0u,
               result.event_id, result.run_board_ncnn ? 1u : 0u,
               red_event::ToString(result.brick_side),
               red_event::ToString(result.avoid_direction));
    }
}

void MaybeCompleteRedEvent()
{
    static bool board_action_seen = false;
    if (IsBoardLocked(s_red_route_result.kind))
    {
        if (Flag.picture != 0)
            board_action_seen = true;
        if (board_action_seen && Flag.picture == 0 &&
            s_red_event_router.CompleteLockedEvent(s_red_route_result.event_id))
        {
            printf("[RED_ROUTE] board event=%u completed by picture state\n",
                   s_red_route_result.event_id);
            s_red_route_result = s_red_event_router.result();
            board_action_seen = false;
        }
        return;
    }
    board_action_seen = false;

    if (g_run_mode == RUN_MODE_TRACK && IsBrickLocked(s_red_route_result.kind))
    {
        const red_event::BrickDetourRuntimeSnapshot snapshot =
            red_event::LoadBrickDetourState();
        if (snapshot.event_id == s_red_route_result.event_id &&
            snapshot.state == red_event::BrickDetourState::kDone &&
            s_red_event_router.CompleteLockedEvent(s_red_route_result.event_id))
        {
            printf("[RED_ROUTE] brick event=%u completed by detour DONE\n",
                   s_red_route_result.event_id);
            s_red_route_result = s_red_event_router.result();
        }
    }
}

void UpdateRedDetourDiagnostic()
{
    const red_event::BrickDetourState previous_state = s_red_brick_detour.state();
    s_red_detour_command = s_red_brick_detour.Update(MonotonicMs());
    if (previous_state != s_red_brick_detour.state())
    {
        printf("[BRICK_DETOUR_DIAG] state=%s direction=%s active=%u ignore_image=%u "
               "speed_valid=%u speed=%.1f dir_offset=%.1f control_consumed=0\n",
               red_event::ToString(s_red_brick_detour.state()),
               red_event::ToString(s_red_brick_detour.direction()),
               s_red_detour_command.active ? 1u : 0u,
               s_red_detour_command.ignore_image_steering ? 1u : 0u,
               s_red_detour_command.speed_override_valid ? 1u : 0u,
               s_red_detour_command.speed_goal, s_red_detour_command.dir_offset);
    }
}

void UpdateCameraFocusStream(const cv::Mat &frame_bgr)
{
    static std::chrono::steady_clock::time_point last_publish;
    const auto now = std::chrono::steady_clock::now();
    const double dt_ms = (last_publish.time_since_epoch().count() == 0) ? 0.0 :
        std::chrono::duration<double, std::milli>(now - last_publish).count();
    if (dt_ms > 0.0 && dt_ms < kFocusPublishPeriodMs)
    {
        return;
    }

    cv::Mat gray;
    cv::Mat laplacian;
    cv::cvtColor(frame_bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Laplacian(gray, laplacian, CV_64F, 3);

    cv::Scalar gray_mean;
    cv::Scalar gray_stddev;
    cv::Scalar lap_mean;
    cv::Scalar lap_stddev;
    cv::meanStdDev(gray, gray_mean, gray_stddev);
    cv::meanStdDev(laplacian, lap_mean, lap_stddev);
    const double sharpness = lap_stddev[0] * lap_stddev[0];

    static double stream_fps = 0.0;
    if (dt_ms > 0.0)
    {
        const double instant_fps = 1000.0 / dt_ms;
        stream_fps = (stream_fps > 0.0) ?
                     (0.8 * stream_fps + 0.2 * instant_fps) : instant_fps;
    }
    last_publish = now;

    cv::Mat display;
    cv::copyMakeBorder(frame_bgr, display, 0, kFocusStatusHeight, 0, kFocusSidePanelWidth,
                       cv::BORDER_CONSTANT, cv::Scalar(18, 18, 18));
    if (s_focus_roi_valid)
    {
        cv::rectangle(display, s_focus_roi_rect, cv::Scalar(0, 255, 0), 2);
        cv::circle(display, s_focus_red_center, 3, cv::Scalar(0, 255, 255), -1);
        const int label_y = (s_focus_roi_rect.y > 12) ? s_focus_roi_rect.y - 4 : 12;
        cv::putText(display, "ROI80",
                    cv::Point(s_focus_roi_rect.x, label_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(0, 255, 0), 1,
                    cv::LINE_AA);
    }
    if (s_focus_direct64_valid)
    {
        cv::rectangle(display, s_focus_direct64_rect, cv::Scalar(0, 255, 255), 2);
        const int label_y = (s_focus_direct64_rect.y > 25)
                                ? s_focus_direct64_rect.y - 16
                                : s_focus_direct64_rect.y + 12;
        cv::putText(display, "ROI64",
                    cv::Point(s_focus_direct64_rect.x, label_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(0, 255, 255), 1,
                    cv::LINE_AA);
    }
    constexpr int kModelPreviewX = 328;
    constexpr int kModelPreviewY = 20;
    cv::putText(display, "MODEL64", cv::Point(kModelPreviewX, 13),
                cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(230, 230, 230), 1,
                cv::LINE_AA);
    if (!s_focus_direct64_input.empty())
    {
        s_focus_direct64_input.copyTo(
            display(cv::Rect(kModelPreviewX, kModelPreviewY,
                             LQ_NCNN::kInputWidth, LQ_NCNN::kInputHeight)));
        cv::rectangle(display,
                      cv::Rect(kModelPreviewX, kModelPreviewY,
                               LQ_NCNN::kInputWidth, LQ_NCNN::kInputHeight),
                      cv::Scalar(255, 255, 255), 1);
        cv::putText(display, "DIRECT64", cv::Point(kModelPreviewX, kModelPreviewY + 76),
                    cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(200, 200, 200), 1,
                    cv::LINE_AA);
    }
    else
    {
        cv::putText(display, "none", cv::Point(kModelPreviewX, kModelPreviewY + 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(160, 160, 160), 1,
                    cv::LINE_AA);
    }
    char line1[96] = {};
    char line2[96] = {};
    char line3[96] = {};
    char line4[96] = {};
    std::snprintf(line1, sizeof(line1), "Sharpness LapVar: %.1f", sharpness);
    std::snprintf(line2, sizeof(line2), "Y: %.1f   stream: %.1f FPS   BGR:320x240",
                  gray_mean[0], stream_fps);
    const bool brick_branch =
        s_red_route_result.kind == red_event::RedEventKind::kBrickLeftCandidate ||
        s_red_route_result.kind == red_event::RedEventKind::kBrickRightCandidate ||
        s_red_route_result.kind == red_event::RedEventKind::kBrickLeftLocked ||
        s_red_route_result.kind == red_event::RedEventKind::kBrickRightLocked;
    if (s_focus_direct64_result_valid)
    {
        std::snprintf(line3, sizeof(line3), "D64:%s conf:%.3f dist(est):%.1fcm",
                      classifier.Label(s_focus_direct64_class_idx),
                      s_focus_direct64_confidence, s_focus_distance_cm);
    }
    else if (s_focus_result_valid)
    {
        std::snprintf(line3, sizeof(line3), "ROI80:%s conf:%.3f dist(est):%.1fcm",
                      classifier.Label(s_focus_class_idx), s_focus_confidence,
                      s_focus_distance_cm);
    }
    else if (brick_branch)
    {
        std::snprintf(line3, sizeof(line3),
                      "Class:skipped brick dist(est):%.1fcm",
                      s_focus_distance_cm);
    }
    else
    {
        std::snprintf(line3, sizeof(line3), "Class:none conf:-- dist(est):--");
    }
    std::snprintf(line4, sizeof(line4),
                  "Route:%s c:%u plan:%s FSM:%s off:%.0f spd:%s%.0f",
                  red_event::ToString(s_red_route_result.kind),
                  static_cast<unsigned>(s_red_route_result.confirm_count),
                  red_event::ToString(s_red_route_result.avoid_direction),
                  red_event::ToString(s_red_brick_detour.state()),
                  s_red_detour_command.dir_offset,
                  s_red_detour_command.speed_override_valid ? "" : "--",
                  s_red_detour_command.speed_override_valid ? s_red_detour_command.speed_goal : 0.0f);
    cv::putText(display, line1, cv::Point(5, frame_bgr.rows + 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(90, 230, 255), 1,
                cv::LINE_AA);
    cv::putText(display, line2, cv::Point(5, frame_bgr.rows + 34),
                cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(230, 230, 230), 1,
                cv::LINE_AA);
    cv::putText(display, line3, cv::Point(5, frame_bgr.rows + 53),
                cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(130, 255, 130), 1,
                cv::LINE_AA);
    cv::putText(display, line4, cv::Point(5, frame_bgr.rows + 72),
                cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(255, 255, 0), 1,
                cv::LINE_AA);
    camera_server.update_frame_mat(display);

    static uint32_t log_counter = 0;
    if ((log_counter++ % 5u) == 0u)
    {
        if (s_focus_direct64_result_valid)
        {
            printf("[FOCUS] sharpness_lap_var=%.1f brightness_y=%.1f stream_fps=%.1f "
                   "direct64_class=%s direct64_confidence=%.3f distance_est_cm=%.1f "
                   "model=current_camera_own6_tiny64_w1_v2 gain=none BGR=320x240\n",
                   sharpness, gray_mean[0], stream_fps,
                   classifier.Label(s_focus_direct64_class_idx),
                   s_focus_direct64_confidence, s_focus_distance_cm);
        }
        else if (s_focus_result_valid)
        {
            printf("[FOCUS] sharpness_lap_var=%.1f brightness_y=%.1f stream_fps=%.1f "
                   "roi80_class=%s roi80_confidence=%.3f distance_est_cm=%.1f "
                   "model=current_camera_own6_tiny64_w1_v2 gain=none BGR=320x240\n",
                   sharpness, gray_mean[0], stream_fps,
                   classifier.Label(s_focus_class_idx), s_focus_confidence,
                   s_focus_distance_cm);
        }
        else if (brick_branch)
        {
            printf("[FOCUS] sharpness_lap_var=%.1f brightness_y=%.1f stream_fps=%.1f "
                   "class=skipped_brick distance_est_cm=%.1f BGR=320x240\n",
                   sharpness, gray_mean[0], stream_fps, s_focus_distance_cm);
        }
        else
        {
            printf("[FOCUS] sharpness_lap_var=%.1f brightness_y=%.1f stream_fps=%.1f "
                   "class=none BGR=320x240\n",
                   sharpness, gray_mean[0], stream_fps);
        }
    }
}

void SaveImageTestCapture(const cv::Mat &frame_bgr,
                          const cv::Mat &candidate_roi80_bgr,
                          const cv::Mat &density_roi64_bgr,
                          const cv::Mat &model_input64_bgr,
                          const cv::Point &raw_center, const cv::Rect &raw_roi,
                          int search_x, int search_y, int search_width, int search_height,
                          int resized_center_x, int resized_center_y,
                          bool white_ratio_valid, float white_ratio,
                          int class_idx, float confidence, long infer_us,
                          const float class_probs[3])
{
    if (g_run_mode != RUN_MODE_IMAGE_TEST ||
        s_image_test_capture_count >= kImageTestCaptureLimit)
    {
        return;
    }
    if ((s_image_test_infer_count++ % kImageTestCaptureInferInterval) != 0)
    {
        return;
    }

    constexpr const char *kCaptureRoot = "/tmp/4m_lq_debug";
    if (mkdir(kCaptureRoot, 0700) != 0 && errno != EEXIST)
    {
        printf("[REC] debug capture mkdir failed: %s\n", strerror(errno));
        return;
    }

    char capture_dir[64] = {};
    snprintf(capture_dir, sizeof(capture_dir), "%s/%d", kCaptureRoot, (int)getpid());
    if (mkdir(capture_dir, 0700) != 0 && errno != EEXIST)
    {
        printf("[REC] debug capture run mkdir failed: %s\n", strerror(errno));
        return;
    }

    const int capture_index = s_image_test_capture_count;
    char frame_path[96] = {};
    char annotated_path[96] = {};
    char candidate_roi_path[112] = {};
    char density_roi_path[112] = {};
    char model_input_path[112] = {};
    char metadata_path[96] = {};
    snprintf(frame_path, sizeof(frame_path), "%s/frame_%02d_bgr.png", capture_dir, capture_index);
    snprintf(annotated_path, sizeof(annotated_path), "%s/frame_%02d_annotated.png", capture_dir, capture_index);
    snprintf(candidate_roi_path, sizeof(candidate_roi_path),
             "%s/candidate_roi80_%02d_bgr.png", capture_dir, capture_index);
    snprintf(density_roi_path, sizeof(density_roi_path),
             "%s/density_roi64_%02d_bgr.png", capture_dir, capture_index);
    snprintf(model_input_path, sizeof(model_input_path),
             "%s/model_input64_%02d_bgr.png", capture_dir, capture_index);
    snprintf(metadata_path, sizeof(metadata_path), "%s/capture_%02d.txt", capture_dir, capture_index);

    cv::Mat annotated = frame_bgr.clone();
    cv::rectangle(annotated, raw_roi, cv::Scalar(0, 255, 0), 1);
    cv::circle(annotated, raw_center, 3, cv::Scalar(0, 255, 255), -1);
    const bool images_saved = cv::imwrite(frame_path, frame_bgr) &&
                              cv::imwrite(annotated_path, annotated) &&
                              cv::imwrite(candidate_roi_path, candidate_roi80_bgr) &&
                              cv::imwrite(density_roi_path, density_roi64_bgr) &&
                              cv::imwrite(model_input_path, model_input64_bgr);
    FILE *metadata = fopen(metadata_path, "w");
    if (!images_saved || metadata == nullptr)
    {
        if (metadata != nullptr)
        {
            fclose(metadata);
        }
        printf("[REC] debug capture write failed: %s\n", strerror(errno));
        return;
    }

    fprintf(metadata,
            "frame_bgr=%s\n"
            "annotated_frame_bgr=%s\n"
            "candidate_roi80_bgr=%s\n"
            "density_roi64_bgr=%s\n"
            "model_input64_bgr=%s\n"
            "frame_size=%dx%d\n"
            "camera_backend=zf_device_uvc\n"
            "camera_source=320x240 YUYV@60 BGR\n"
            "red_detector=fullres RGB dominance R>%d R-G>%d R-B>%d\n"
            "red_search_resized=%d,%d,%d,%d\n"
            "red_center_resized=%d,%d\n"
            "red_center_raw=%d,%d\n"
            "physical_roi_raw=%d,%d,%d,%d\n"
            "capture_infer_interval=%d\n"
            "physical_roi_contract=80x80 at (cx-40+sx,cy-63+sy), red region included\n"
            "density_roi_contract=direct 64x64 at (cx-32+sx,cy-50+sy) for white_ratio only\n"
            "model_input_contract=physical ROI80 resized to 64x64 BGR with INTER_LINEAR\n"
            "roi_shift=%d,%d\n"
            "backend=ncnn\n"
            "model_source=provider Loongson_2K300_301_LIB (2)/model\n"
            "model_id=%s\n"
            "model_param=%s\n"
            "model_param_expected_sha256=%s\n"
            "model_bin=%s\n"
            "model_bin_expected_sha256=%s\n"
            "model_input=%s\n"
            "white_ratio_valid=%d\n"
            "white_ratio=%.6f\n"
            "white_ratio_runtime_threshold=%.2f\n"
            "white_ratio_policy=>=threshold board; <threshold brick\n"
            "model_graph=in0 NCHW [1,3,64,64] -> out0 [1,3] raw logits\n"
            "runtime_postprocess=stable softmax probabilities [0,1]\n"
            "output_order=supplies,vehicle,weapon\n"
            "probs=%.9f,%.9f,%.9f\n"
            "class_idx=%d\n"
            "confidence=%.9f\n"
            "infer_us=%ld\n"
            "conf_gate=%.2f\n",
            frame_path, annotated_path, candidate_roi_path, density_roi_path,
            model_input_path,
            frame_bgr.cols, frame_bgr.rows,
            Flash.debug_rgb_r_min, Flash.debug_rgb_rg_diff, Flash.debug_rgb_rb_diff,
            search_x, search_y, search_width, search_height,
            resized_center_x, resized_center_y,
            raw_center.x, raw_center.y,
            raw_roi.x, raw_roi.y, raw_roi.width, raw_roi.height,
            kImageTestCaptureInferInterval,
            ROI_SHIFT_X, ROI_SHIFT_Y,
            LQ_NCNN::ModelId(),
            LQ_NCNN::ParamFileName(), LQ_NCNN::ExpectedParamSha256(),
            LQ_NCNN::BinFileName(), LQ_NCNN::ExpectedBinSha256(),
            LQ_NCNN::PreprocessDescription(),
            white_ratio_valid ? 1 : 0,
            white_ratio_valid ? white_ratio : -1.0f,
            s_red_event_router.config().white_ratio,
            class_probs[0], class_probs[1], class_probs[2],
            class_idx, confidence, infer_us, g_picture_conf_min);
    fclose(metadata);

    s_image_test_capture_count++;
    // 每轮进程独立目录，避免旧 roi_xx 混入统计；日志打印完整路径便于 scp
    if (capture_index == 0)
    {
        printf("[REC] debug capture dir (this run only): %s\n", capture_dir);
    }
    printf("[REC] debug capture %d/%d saved: %s\n",
           capture_index, kImageTestCaptureLimit - 1, metadata_path);
}

}
//绕行相关
bool picture_yaw_init = false;//记录第一次进入绕行逻辑的标志位
float Yaw_picture = 0;//记录检测到图片时的初始yaw值
float Yaw_picture_diff = 0;//当前的YAW值相较于Yaw_picture的差值
float Yaw_picture_err = 0;
float Yaw_picture_target = 0;//绕行的目标偏差值
float encoder_val = 0;//用于记录编码器的值 以实现分阶段运行
int resize_cx,resize_cy = 0;
//处理陀螺仪角度跳变
float Yaw_correct(float current_yaw,float target_yaw)
{
    float diff = current_yaw - target_yaw;
    if(diff >=180)
    {
        diff =  diff - 360;
    }
    if (diff<=-180)
    {
        diff =  diff + 360;
    }
    return diff;

}

const float atan_deg_tab[49] = 
{
    0.00f, 45.00f, 63.43f, 71.57f, 75.96f, 78.69f, 80.54f, 81.87f, 82.87f, 83.66f,
    84.29f, 84.81f, 85.24f, 85.60f, 85.91f, 86.19f, 86.42f, 86.63f, 86.82f, 86.99f,
    87.14f, 87.27f, 87.40f, 87.51f, 87.61f, 87.71f, 87.80f, 87.88f, 87.95f, 88.03f,
    88.09f, 88.15f, 88.21f, 88.26f, 88.32f, 88.36f, 88.41f, 88.45f, 88.49f, 88.53f,
    88.57f, 88.60f, 88.64f, 88.67f, 88.70f, 88.73f, 88.75f, 88.78f, 88.81f
};
float actan_err(float err)
{
    float actan_err;
    if(err>0) actan_err=atan_deg_tab[(int)err];
    if(err<=0) actan_err=-atan_deg_tab[-(int)err];
  return actan_err;
}
// float real_distance[60] =
// {
//      536.00, 413.00, 330.50, 276.40, 235.00, 205.80, 179.20, 156.40, 142.40, 132.40,
//      118.20, 107.40, 99.00, 91.60, 83.50, 78.60, 73.70,  68.00,  62.40,  56.60,
//      54.80,  51.00,  47.00,  44.00,  42.00,  39.00,  36.20,  34.00,  31.50,  30.40,
//      28.20,  26.60,  24.80,  23.20,  21.90,  20.60,  19.40,  18.40,  17.40,  16.10,
//      15.20,  14.20,  13.40,  12.80,  12.00,  11.20,  10.60,  9.80,  8.80,  8.00, 
//      7.20,  6.70,  6.20,  5.40,  4.60,  4.00,  3.50,  2.80,  2.00,  1.20,
// };

// float real_distance[60] =
// {
//      750.00-3,590.00-3, 460.00-3, 366.00-3, 301.50-3, 262.00-3, 226.00-3, 199.00-3, 177.00-3, 160.0-3,
//      144.00-3, 132.00-3, 122.00-3, 113.0-30, 103.90-3, 97.00-3, 90.00-3,  82.50-3,  78.00-3,  73.00-3,
//      67.50-3,  64.00-3,  60.30-3,  55.80-3,  53.40-3,  49.90-3,  46.00-3,  44.30-3,  41.90-3,  38.90-3,
//      37.10-3,  35.50-3,  33.00-3,  31.80-3,  30.10-3,  28.00-3,  26.60-3,  25.40-3,  23.00-3,  22.10-3,
//      21.20-3,  19.80-3,  18.80-3,  18.00-3,  16.00-3,  15.20-3,  14.50-3,  13.00-3,  12.80-3,  12.00-3, 
//      10.70-3,  9.60-3,  9.10-3,  8.80-3,  8.00-3,  7.10-3,  6.50-3,  6.00-3,  5.00-3,  3.80-3,
// };

// float real_distance[60] =
// {
//      500,410, 350, 300.0, 268.0, 226.5, 193.0, 172.0, 153.5, 138.6,
//      126.0, 115.0, 106.2, 97.8, 90.6, 84.6, 79.0,  73.8,  68.8,  64.9,
//      60.8,  58.0,  54.0,  51.0,  48.0,  45.7,  42.5,  40.0,  38.0,  36.5,
//      34.5,  32.4,  30.8,  29.5,  28.1,  26.3,  25.0,  24.0,  22.3,  21.2,
//      20.4,  19.2,  18.0,  17.0,  16.00,  15.20,  14.4,  13.6,  13.0,  12.0, 
//      11.2,  10.6,  10.0,  9.5,  8.7,  8.2,  7.8,  7,  6.5,  3.80-3,
// };

float real_distance[60] =
{
     485,410, 345, 292.0, 244.5, 213.8, 184.0, 161.0, 141.5, 127.0,
     115.0, 104.5, 95.6, 88.0, 82.0, 77.0, 71.7,  67.0,  62.5,  58.0,
     54.7,  51.50,  48.2,  45.3,  42.5,  40.3,  38.0,  36.5,  34.6,  32.5,
     31.0,  29.5,  27.2,  26.5,  24.8,  23.3,  22.1,  21.0,  20.0,  19.0,
     18.0,  17.0,  16.0,  14.9,  14.0,  13.10,  12.6,  12.0,  11.0,  10.5, 
     10.0,  9.3,  8.7,  8.2,  7.5,  6.9,  6.5,  6.0,  5.5,  5,
};

int real_distance_to_row(float distance) {
    float min_diff = 10000.0f;  // 初始化为一个很大的数
    int nearest_index = 0;
    
    for (int i = 0; i < 60; i++) {
        float diff = fabsf(real_distance[i] - distance);
        if (diff < min_diff) {
            min_diff = diff;
            nearest_index = i;
        }
    }
    
    return nearest_index;
}

void imgInfoInit(void)
{
    imgInfo.bottom = LCDH_1 - 1;
    imgInfo.lastMid = LCDW_1 / 2;
    imgInfo.top = 0;
    imgInfo.L_loselineSum = 0;
    imgInfo.R_loselineSum = 0;
}

// 分类模型只在 image_init() 中初始化，运行时不设置第二后端或 fallback。
//-------------------------------------------------------------------------------------------------------------------
//  @brief      优化的大津法
//  @param      image  图像数组
//  @param      clo    宽
//  @param      row    高
//  @param      pixel_threshold 阈值分离
//  @return     uint8
//  @since      2021.6.23
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8_t Threshold_deal(uint8 image[LCDH_1][LCDW_1],uint16 col,uint16 row) {
  #define GrayScale 255
  uint16 width = col;
  uint16 height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
//  uint8 threshold = 0;
  uint8* data =(uint8*)  image;  //指向像素数据的指针
  for (i = 0; i < GrayScale; i++) {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0;
  //统计灰度级中 每个像素在整幅图像中的个数
  for (i = 0; i < height; i += 1) {
    for (j = 0; j < width; j += 1) {
      // if((sun_mode&&data[i*width+j]<pixel_threshold)||(!sun_mode))
      //{
      pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
      gray_sum += (int)data[i * width + j];  //灰度值总和
      //}
    }
  }

  //计算每个像素值的点在整幅图像中的比例
  for (i = 0; i < GrayScale; i++) {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }


  //遍历灰度级[0,255]
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; j < 180; j++) {
    w0 +=
        pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和 即背景部分的比例
    u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;    //背景平均灰度
    u1 = u1tmp / w1;    //前景平均灰度
    u = u0tmp + u1tmp;  //全局平均灰度
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax) {
      deltaMax = deltaTmp;
      Threshold = (uint8)j;
    }
    if (deltaTmp < deltaMax) break;
  }

  return (uint8)Threshold;
}


/******************************************************图像二值化************************************************/


// 二值下限：大津结果 < 此值则抬到此值。作者 70；现默认 35，可用参数 threshold_static 热改。
uint8 Threshold_static = 35;
void Get01change_dajin() {
//    if(run_flag==0){
        Threshold = Threshold_deal(Image_Use, LCDW_1, LCDH_1);
//    }
  if (Threshold < Threshold_static)
    Threshold = (uint8)Threshold_static;
  uint8 i, j = 0;
  int thre;
  for (i = 0; i < LCDH_1; i++) {
    for (j = 0; j < LCDW_1; j++) {
      if (j <= 18)
        thre = Threshold - 10;
      else if ((j > 82 && j <= 88))
        thre = Threshold - 10;
      else if (j >= 76)
        thre = Threshold - 10;
      else
        thre = Threshold;

      if (Image_Use[i][j] >(thre))         //数值越大，显示的内容越多，较浅的图像也能显示出来
          Image_Use[i][j] = 255;  //白
      else
          Image_Use[i][j] = 0;  //黑
    }
  }
//     for (i = 50; i < 60; i++) {
//     for (j = LCDW_1/2-15; j < LCDW_1/2+15; j++) {

//           Image_Use[i][j] = 255;  //白

//     }
//   }

    // //图像最小范围 35~60
    // int roi_resize_x = roix1*0.5;
    // int roi_resize_y = roiy1*0.5;
    // int roi_y_2 = roi_resize_y + 25;
    // int roi_x_2 = roi_resize_x + 25;
    // if(roi_resize_x < 35) roi_resize_x = 35;//确保在赛道范围内 同时不要越界
    // if(roi_x_2 > 65) roi_x_2 = 65;

    // if(roi_y_2 > 60) roi_y_2 = 60;
    // if(roi_y_2<0)   roi_y_2 = 0;
    
    // if(Flag.Redblock == 1)
    // {
    //     for(int i = roi_resize_y;i<roi_y_2;i++)
    //     {
    //         for(int j = roi_resize_x;j<roi_x_2;j++)
    //         {
    //             Image_Use[i][j] = 255;
    //         }
    //     }
    // }

}


void my_sobel(unsigned char imageIn[LCDH_1][LCDW_1], unsigned char imageOut[LCDH_1][LCDW_1])
{
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = LCDW_1 - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = LCDH_1 - KERNEL_SIZE / 2;
    short i, j;
    short temp[2];
    short temp1 ,temp2 ;
    //for(i = 0; i < Compress_H; i++)//算的更慢不过对比了全局图像
    for (i = yStart; i < yEnd; i++)   //有点的跳跃
       {
           //for(j = 0; j < Compress_W; j++)//算的更慢不过对比了全局图像
           for (j = xStart; j < xEnd; j++)  //有点的跳跃
           {
               /* 计算不同方向梯度幅值  */
               temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
               - (short) 2*imageIn[i][j - 1] + (short) 2*imageIn[i][j + 1]       // {-2, 0, 2},
               - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

               temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -2, -1},
               - (short) 2*imageIn[i - 1][j] + (short) 2*imageIn[i + 1][j]       // { 0,  0,  0},
               - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  2,  1}};

               temp[0] = fabs(temp[0]);
               temp[1] = fabs(temp[1]);

               temp1 = temp[0] + temp[1] ;

               temp2 =  (short) imageIn[i - 1][j - 1] + (short)2* imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
                       + (short)2* imageIn[i][j - 1] + (short) imageIn[i][j] + (short) 2*imageIn[i][j + 1]
                       + (short) imageIn[i + 1][j - 1] + (short) 2*imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

               if (temp1 > temp2 / 12.0f)
               {
                   imageOut[i][j] = black;
               }
              else
              {
                  imageOut[i][j] = white;
              }
           }
       }

}

/******************************************************图像二值化************************************************/




void my_sobel_dajin(unsigned char imageIn[LCDH_1][LCDW_1], unsigned char imageOut[LCDH_1][LCDW_1])
{       

    //    if(run_flag==0){
        Threshold = Threshold_deal(Image_Use, LCDW_1, LCDH_1);
//    }
  if (Threshold < Threshold_static)
    Threshold = (uint8)Threshold_static;

    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = LCDW_1 - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = LCDH_1 - KERNEL_SIZE / 2;
    short i, j;
    short temp[2];
    short temp1 ,temp2 ;
    //for(i = 0; i < Compress_H; i++)//算的更慢不过对比了全局图像
    for (i = yStart; i < yEnd; i++)   //有点的跳跃
       {
           //for(j = 0; j < Compress_W; j++)//算的更慢不过对比了全局图像
           for (j = xStart; j < xEnd; j++)  //有点的跳跃
           {
               /* 计算不同方向梯度幅值  */
               temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
               - (short) 2*imageIn[i][j - 1] + (short) 2*imageIn[i][j + 1]       // {-2, 0, 2},
               - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

               temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -2, -1},
               - (short) 2*imageIn[i - 1][j] + (short) 2*imageIn[i + 1][j]       // { 0,  0,  0},
               - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  2,  1}};

               temp[0] = fabs(temp[0]);
               temp[1] = fabs(temp[1]);

               temp1 = temp[0] + temp[1] ;

            //    temp2 =  (short) imageIn[i - 1][j - 1] + (short)2* imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
            //            + (short)2* imageIn[i][j - 1] + (short) imageIn[i][j] + (short) 2*imageIn[i][j + 1]
            //            + (short) imageIn[i + 1][j - 1] + (short) 2*imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

               if (temp1 > Threshold*2.0)
               {
                   imageOut[i][j] = 0;
               }
              else
              {
                  imageOut[i][j] = 255;
              }
           }
       }

  int thre;
  for (i = 0; i < LCDH_1; i++) {
    for (j = 0; j < LCDW_1; j++) {
      if (j <= 18)
        thre = Threshold - 10;
      else if ((j > 82 && j <= 88))
        thre = Threshold - 10;
      else if (j >= 76)
        thre = Threshold - 10;
      else
        thre = Threshold;

      if (imageIn[i][j] <(thre))         //数值越大，显示的内容越多，较浅的图像也能显示出来
          imageOut[i][j] = 0;  //hei
    //   else
    //    imageOut[i][j] = 1;  //hei
    }
  }
}


/***************************************************给图像两边画黑框*************************************************/

void Draw_BlackSideline(uint8_t Image_1[LCDH_1][LCDW_1])
{
    int i = 0;
        for(i = imgInfo.bottom - 1; i > imgInfo.top; i--)
        {
            Image_1[i][0] = black;
            Image_1[i][LCDW_1 - 1] = black;
        }
}
/***************************************************获取最长白列***************************************************/

int Mid_Line[LCDH_0] = {LCDW_0 / 2};     // 中线数组，初始化所有元素为屏幕宽度的一半
int Last_Mid_Line[LCDH_0] = {0};           // 上一次中线数组，初始化所有元素为0

/* 获取截止行和最长白列 */
void Get_ImageTop(void)
{

    int8 len;                            // 临时变量，用于存储白色像素列的长度
    uint8 j;                             // 循环变量，用于遍历列
    imgInfo.white_num = 0;               // 初始化最长白色像素列的长度为0
    imgInfo.top = 0;                     // 初始化图像的有效顶部行号为0
    imgInfo.max_column = 0;              // 初始化最长白色像素列的列号为0

    uint8 l_side=0 , r_side=0 ;                    // 默认左边界（对应循环终止条件 i==2）
                                                   // 默认右边界（对应循环终止条件 i==LCDW_1-3）

    static uint8 flag_m = 0;             // 静态标志变量，用于判断是否是第一次执行

    if(flag_m == 0)                      // 如果是第一次执行
    {
        Last_Mid_Line[imgInfo.bottom - 1] = LCDW_1 / 2; // 设置上一次中线位置为屏幕长度的一半
        flag_m = 1;                      // 将标志变量设置为1，表示已执行过一次
    }

//    if(Mode_1)                           // 如果处于模式1
//    {
        imgInfo.bottom = LCDH_1 - 1;     // 设置图像的底部行号为屏幕高度减1

        if(Last_Mid_Line[imgInfo.bottom - 1] > LCDW_1 - 1 || Last_Mid_Line[imgInfo.bottom - 1] < 0)
        {                                // 如果上一次中线位置超出屏幕宽度范围
            Last_Mid_Line[imgInfo.bottom - 1] = LCDW_1 / 2; // 重新设置上一次中线位置为屏幕宽度的一半
        }

        // 从上一次中线位置向左搜索白色区域的左边界
        for(int i = Last_Mid_Line[imgInfo.bottom - 1]; i > 1; i--)
        {
           if((Image_Use[imgInfo.bottom - 1][i] == white
               && Image_Use[imgInfo.bottom - 1][i - 1] == black
               && Image_Use[imgInfo.bottom - 1][i - 2] == black)
                   || (i==2))//寻找到白黑黑跳变点 或者找到最左边（会在图像四周画上图像，所以i=2为最左边）
           {
               l_side = (uint8)i;               // 找到左边界，退出循环
               break;
           }
        }

        // 从上一次中线位置向右搜索白色区域的右边界
        for(int i = Last_Mid_Line[imgInfo.bottom - 1]; i < LCDW_1 - 2; i++)
        {
           if((Image_Use[imgInfo.bottom - 1][i] == white
               && Image_Use[imgInfo.bottom - 1][i + 1] == black
               && Image_Use[imgInfo.bottom - 1][i + 2] == black)
                   || (i==LCDW_1 - 3))
           {
               r_side =(uint8) i;               // 找到右边界，退出循环
               break;
           }
        }
            // 在左右边界之间，每隔4列检查一次最长白色像素列
            for (j = l_side; j <= r_side; j += 1)
            {
                for (len = imgInfo.bottom - 1; len >= imgInfo.top && Image_Use[len][j]; len--);
                                             // 从底部向上搜索，直到找到非白色像素或到达顶部

                len = LCDH_1 - len;          // 计算白色像素列的长度（从底部到顶部的距离）
                if (imgInfo.white_num < len)
                {
                    imgInfo.white_num = len; // 更新最长白色像素列的长度
                    imgInfo.max_column = j;  // 更新最长白色像素列的列号
                }
            }
        // 确定截止行（图像有效部分的顶部）
        if ((LCDH_1 - imgInfo.white_num) >= 0)
            imgInfo.top = LCDH_1 - imgInfo.white_num + 1; // 根据最长白色像素列的长度计算截止行
        else
            imgInfo.top = 0;             // 如果计算出的截止行小于0，则设置为0
//    }
}

int Find_Top(int line)
{
    for(int i = imgInfo.bottom - 1; i <= imgInfo.top; i--)
    {
        if(Image_Use[i - 1][line] == white && Image_Use[i][line] == black && Image_Use[i + 1][line] == black)
        {
            return i;
        }
    }
    return 0;
}


/**
 * @name: xielv_sideline
 * @details: 计算斜率
 * @param {uint8 x1, uint8 y1    第一个点的坐标
 *         uint8 x2, uint8 y2    第二个点的坐标
 *         uint8 data            返回的值 斜率 k 或 常数 b，只能给大小写的k或b，给其他值则返回k
 *          }
 * @return: 返回的值 斜率 k 或 常数 b
 **/
float xielv_sideline(int x1, int y1, int x2, int y2, char data)
{
    float k = 0.0, b = 0.0;

    k = (float)(y2 - y1) / (x2 - x1);
    b = (float)(y1 - k * x1);

    if (data == 'b' || data == 'B')
        return b;
    else if (data == 'k' || data == 'K')
        return k;
    else
        return k;
}

int Left_Sideline[LCDH_0] = {0};    //左边线数组
unsigned char Left_Sideline_flag[LCDH_0] = {0};   //左边线标志位

int Right_Sideline[LCDH_0] = {LCDH_0 - 2};   //右边线数组
unsigned char Right_Sideline_flag[LCDH_0] = {0};  //右边线标志位

unsigned char white_width[LCDH_0] = {0}; //每行的白点数（宽度）
unsigned char starith_white_width[LCDH_0] = {0}; //每行的白点数（宽度）
/* 寻找边线 */

void Find_Sideline(uint8 Start_row, uint8 End_row)
{
    int i = 0, j = 0;
    imgInfo.L_loselineSum = 0; // 初始化左侧丢失线（未检测到边线）的计数器为0

//    else if(Mode_1) // 如果处于模式1
//    {
        for(i = Start_row; i >= End_row; i--) // 从起始行向结束行遍历（逆序）
        {
            Left_Sideline_flag[i] = 0; // 初始化当前行的左侧边线标志为0（未检测到）
            Right_Sideline_flag[i] = 0; // 初始化当前行的右侧边线标志为0（未检测到）

            if(Image_Use[i][imgInfo.max_column] == black) // 如果最长白列的当前行像素为黑色（可能是阴影或障碍物）
            {
                Left_Sideline[i] = Left_Sideline[i + 1]; // 使用上一行的左侧边线位置
                Right_Sideline[i] = Right_Sideline[i + 1]; // 使用上一行的右侧边线位置
                break; // 跳出循环，因为当前行被认为是无效的
            }

            // 左边线检测 从最长白列往左找线
            for(j = imgInfo.max_column; j > 1; j--)
            {
                /* 正常白黑黑跳变点 或者白黑白但距离上一次记录的左侧边线位置小于3列，则认为找到了左侧边线 因为用的压缩图像 可能会有噪点*///
                if((Image_Use[i][j] == white && Image_Use[i][j - 1] == black && Image_Use[i][j - 2] == black)//&& (i<= (imgInfo.top+10)||(i> (imgInfo.top+10)&&(j - Left_Sideline[i+1] < 30)))
                    || (Image_Use[i][j] == white && Image_Use[i][j - 1] == black && Image_Use[i][j - 2] == white && j - Left_Sideline[i+1] < 3))//
                {
                    Left_Sideline[i] = j; // 记录左侧边线的列号
                    Left_Sideline_flag[i] = 1; // 设置左侧边线标志为1（已检测到）
                    break; // 找到左侧边线后退出循环
                }
            }
            if(j == 1 && Left_Sideline_flag[i] == 0) // 如果遍历到第一列仍未检测到左侧边线
            {
                Left_Sideline[i] = 1; // 将左侧边线设置为第一列
                Left_Sideline_flag[i] = 0; // 左侧边线标志保持为0（未真实检测到）
                imgInfo.L_loselineSum ++; // 左侧丢失线计数器加1
            }

            for(j = imgInfo.max_column; j < LCDW_1 - 2; j++)//
            {
                if((Image_Use[i][j] == white && Image_Use[i][j + 1] == black && Image_Use[i][j + 2] == black)//&& (i<=(imgInfo.top+10)||(i> (imgInfo.top+10)&&(Right_Sideline[i + 1] - j < 30)))
                    || (Image_Use[i][j] == white && Image_Use[i][j + 1] == black && Image_Use[i][j + 2] == white && Right_Sideline[i + 1] - j < 3))
                {
                    Right_Sideline[i] = j; // 记录右侧边线的列号
                    Right_Sideline_flag[i] = 1; // 设置右侧边线标志为1（已检测到）
                    break; // 找到右侧边线后退出循环
                }
            }
            if(j == LCDW_1 - 2 && Right_Sideline_flag[i] == 0) // 如果遍历到最后一列前两列仍未检测到右侧边线
            {
                Right_Sideline[i] = LCDW_1 - 2; // 将右侧边线设置为最后一列前两列（通常是屏幕边界前的安全位置）
                Right_Sideline_flag[i] = 0; // 右侧边线标志保持为0（未真实检测到）
                imgInfo.R_loselineSum ++; // 右侧丢失线计数器加1
            }

            white_width[i] = Right_Sideline[i] - Left_Sideline[i]; // 计算并记录当前行的白色区域宽度
        }
      //  printf("Before Avoid: %d\n", Left_Sideline[40]);
//    }
}

void Find_left_Sideline(uint8 Start_row, uint8 End_row)
{
    imgInfo.L_loselineSum = 0; // 初始化左侧丢失线（未检测到边线）的计数器为0
    int i = 0, j = 0;

        for(i = Start_row; i >= End_row; i--) // 从起始行向结束行遍历（逆序）
        {
                       if(Image_Use[i][Left_Sideline[i+1]+7] == white)
            {
            Left_Sideline_flag[i] = 0; // 初始化当前行的左侧边线标志为0（未检测到）

            // if(Image_Use[i][max_column] == black) // 如果最长白列的当前行像素为黑色（可能是阴影或障碍物）
            // {
            //     Left_Sideline[i] = Left_Sideline[i + 1]; // 使用上一行的左侧边线位置
            //     break; // 跳出循环，因为当前行被认为是无效的
            // }

            // 左边线检测 从最长白列往左找线
 
            for(j = Left_Sideline[i+1]+7; j > 1; j--)
            {
                /* 正常白黑黑跳变点 或者白黑白但距离上一次记录的左侧边线位置小于3列，则认为找到了左侧边线 因为用的压缩图像 可能会有噪点*///
                if((Image_Use[i][j] == white && Image_Use[i][j - 1] == black && Image_Use[i][j - 2] == black)//&& (i<= (imgInfo.top+10)||(i> (imgInfo.top+10)&&(j - Left_Sideline[i+1] < 30)))
                    || (Image_Use[i][j] == white && Image_Use[i][j - 1] == black && Image_Use[i][j - 2] == white && j - Left_Sideline[i+1] < 3))//
                {
                    Left_Sideline[i] = j; // 记录左侧边线的列号
                    Left_Sideline_flag[i] = 1; // 设置左侧边线标志为1（已检测到）
                    break; // 找到左侧边线后退出循环
                }
            }
            if(j == 1 && Left_Sideline_flag[i] == 0) // 如果遍历到第一列仍未检测到左侧边线
            {
                Left_Sideline[i] = 1; // 将左侧边线设置为第一列
                Left_Sideline_flag[i] = 0; // 左侧边线标志保持为0（未真实检测到）
            }
           }

        }
        for(int i=imgInfo.bottom-1;i>=(imgInfo.top+ 1);i--)
        {
            if( Left_Sideline_flag[i] ==0)              
              imgInfo.L_loselineSum ++; // 左侧丢失线计数器加1
             white_width[i] = Right_Sideline[i] - Left_Sideline[i]; // 计算并记录当前行的白色区域宽度
        }
}

void Find_right_Sideline(uint8 Start_row, uint8 End_row)
{
    imgInfo.R_loselineSum = 0; // 初始化右侧丢失线计数器为0
    int i = 0, j = 0;

    for(i = Start_row; i >= End_row; i--) // 从起始行向结束行逆序遍历
    {
        // ===================== 修复 1：防止 j 为负数（最关键！）=====================
        int start_j = Right_Sideline[i+1] - 7;
        if (start_j < 0) start_j = 0;  // 强制 >=0，不越界

        if(Image_Use[i][start_j] == white)
        {
            Right_Sideline_flag[i] = 0;

            // ===================== 修复 2：安全起始点，不会负数 =====================
            for(j = start_j; j < LCDW_1 - 2; j++)
            {
                // ===================== 修复 3：右边线正确跳变判断 =====================
                if((Image_Use[i][j] == white && Image_Use[i][j + 1] == black && Image_Use[i][j + 2] == black)
                    || (Image_Use[i][j] == white && Image_Use[i][j + 1] == black && Image_Use[i][j + 2] == white && Right_Sideline[i + 1] - j < 3))
                {
                    Right_Sideline[i] = j;
                    Right_Sideline_flag[i] = 1;
                    break;
                }
            }

            // ===================== 修复 4：统一的未检测到边界处理 =====================
            if(j >= LCDW_1 - 2 && Right_Sideline_flag[i] == 0)
            {
                Right_Sideline[i] = LCDW_1 - 2;
                Right_Sideline_flag[i] = 0;
            }
        }
    }

    // ===================== 修复 5：丢行统计 移到外面（和左边完全一致）=====================
    for(int i = imgInfo.bottom - 1; i >= imgInfo.top + 1; i--)
    {
        if(Right_Sideline_flag[i] == 0)
            imgInfo.R_loselineSum ++;

        white_width[i] = Right_Sideline[i] - Left_Sideline[i];
    }
}

struct Guaidian L_l_guai, L_h_guai, R_l_guai, R_h_guai;  //拐点信息结构体
int Guai_row = 0;

void Find_Guaidian(void)
{
    L_l_guai.flag = 0;
    L_h_guai.flag = 0;
    R_l_guai.flag = 0;
    R_h_guai.flag = 0;


    if(L_l_guai.flag == 0)
        L_l_guai.row = LCDH_1 - 1;
    if(R_l_guai.flag == 0)
        R_l_guai.row = LCDH_1 - 1;

    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {

        /**************左上拐点**************/
        if(
             Left_Sideline[i] > 3 &&
             (
                  /*    *0
                      0100
                           */
                 (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
                  && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0000
                           */
                  (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                   && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && !Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      00010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0]&& Image_Use[i + 1][Left_Sideline[i] + 1] && !Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
//                   ||/*    *0
//                          01000
//                            */
// (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
// && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 3]
// && Image_Use[i + 1][Left_Sideline[i] - 2])
               
             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && xielv_sideline(i, Left_Sideline[i], i + 1, Left_Sideline[i + 1], 'k') < 1
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Left_Sideline[i - 1] - Left_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Left_Sideline[i], i - 3, Left_Sideline[i - 3], 'k') - xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k')) > 0.5
//             && xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k') < 1
             && L_h_guai.flag == 0
             && i < L_l_guai.row
        )
        {
            L_h_guai.row = i + 1;
            L_h_guai.column = (uint8)Left_Sideline[i];
            L_h_guai.flag = 1;

//            if(R_h_guai.flag)
//                break;

        }

        /**************右上拐点**************/
        if(
             Right_Sideline[i] < LCDW_1 - 3 &&
             (
                   /*  0*
                       0010
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && !Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*   
                       0100
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && !Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                      01000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 1] && !Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
//                    ||/*    0*
//                          00010
//                     */
// (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0]
// && Image_Use[i + 1][Right_Sideline[i] + 1] && !Image_Use[i + 1][Right_Sideline[i] + 3]
// && Image_Use[i + 1][Right_Sideline[i] + 2])

             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Right_Sideline[i - 1] - Right_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
             && xielv_sideline(i, Right_Sideline[i], i + 1, Right_Sideline[i + 1], 'k') > -1
             && R_h_guai.flag == 0
             && i < R_l_guai.row
           )
        {
            R_h_guai.row = i + 1;
            R_h_guai.column = (uint8)Right_Sideline[i];
            R_h_guai.flag = 1;

//            if(L_h_guai.flag)
//                break;
        }

    }


//    else if(Mode_1)
//    {
        //找左下，右下拐点
        for(int i = imgInfo.bottom - 3; i >= imgInfo.top + 3 && Flag.Huandao_L != 3 && Flag.Huandao_R != 3; i--)
        {
            if(L_h_guai.flag == 0)
                L_h_guai.row = 1;
            if(R_h_guai.flag == 0)
                R_h_guai.row = 1;
            /**************左下拐点**************/
            if(
                 Left_Sideline[i] > 3 &&
                 (
                      /*  0000
                            *0
                                */
                     (Image_Use[i][Left_Sideline[i] +0] && Image_Use[i - 1][Left_Sideline[i] + 0]
                      && Image_Use[i - 1][Left_Sideline[i] - 1] && Image_Use[i - 1][Left_Sideline[i] - 2]  && Image_Use[i - 1][Left_Sideline[i] -3])
                     ||
                     /*  0100
                           *0
                               */
                     (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] - 1]
                      && !Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                      ||
                      /*  0010
                            *0
                                */
                      (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && !Image_Use[i - 1][Left_Sideline[i] - 1]
                       && Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] - white_width[i + 1] > 10
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i] && white_width[i - 4] > white_width[i]
                 && i > L_h_guai.row
                 && L_l_guai.flag == 0
            )
            {
                L_l_guai.row = i - 1;
                L_l_guai.column = (uint8)Left_Sideline[i];
                L_l_guai.flag = 1;

//                if(R_l_guai.flag)
//                    break;

            }

            /**************右下拐点**************/
            if(
                 Right_Sideline[i] < LCDW_1 - 3 &&
                 (
                         /*
                              0000
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0]
                         && Image_Use[i - 1][Right_Sideline[i] + 1] && Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                         ||
                         /*
                              0010
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                          && !Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                          ||
                          /*
                               0001
                               0*
                           */
                          (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                           && Image_Use[i - 1][Right_Sideline[i] + 2] && !Image_Use[i - 1][Right_Sideline[i] + 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i]
                 && white_width[i - 2] - white_width[i + 1] > 10
//                 && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
//                 && xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') > 1
                 && i > R_h_guai.row
                 && R_l_guai.flag == 0
            )
            {
                R_l_guai.row = i - 1;
                R_l_guai.column = (uint8)Right_Sideline[i];
                R_l_guai.flag = 1;

//                if(L_l_guai.flag)
//                    break;
            }
        }
    
//    }
int l_guai_max=0,r_guai_max=93;//,l_guai_line,r_guai_line
if(Flag.Huandao_R == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Left_Sideline[i]> l_guai_max)//
            {
                l_guai_max = Left_Sideline[i];
                L_l_guai.row = i ;
            }
    }

                L_l_guai.column = (uint8)Left_Sideline[L_l_guai.row]+3;
                L_l_guai.flag = 1;
}
if(Flag.Huandao_L == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Right_Sideline[i]< r_guai_max)//
            {
                r_guai_max = Right_Sideline[i];
                R_l_guai.row = i ;
            }
    }

                R_l_guai.column = (uint8)Right_Sideline[R_l_guai.row]-3;
                R_l_guai.flag = 1;
}
        // for(int i = imgInfo.bottom - 3; i >= imgInfo.top + 3 && Flag.Huandao_L != 3 && Flag.Huandao_R != 3; i--)



}


void Find_Guaidian1(void)
{
                        // printf("Fla:%d\n", white_width[37 + 2] - white_width[37 - 0]);
    L_l_guai.flag = 0;
    L_h_guai.flag = 0;
    R_l_guai.flag = 0;
    R_h_guai.flag = 0;


    if(L_l_guai.flag == 0)
        L_l_guai.row = LCDH_1 - 1;
    if(R_l_guai.flag == 0)
        R_l_guai.row = LCDH_1 - 1;

    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {

        /**************左上拐点**************/
        if(
             Left_Sideline[i] > 3 &&
             (
                  /*    *0
                      0100
                           */
                 (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
                  && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0000
                           */
                  (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                   && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && !Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      00010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0]&& Image_Use[i + 1][Left_Sideline[i] + 1] && !Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
//                   ||/*    *0
//                          01000
//                            */
// (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
// && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 3]
// && Image_Use[i + 1][Left_Sideline[i] - 2])
               
             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && xielv_sideline(i, Left_Sideline[i], i + 1, Left_Sideline[i + 1], 'k') < 1
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Left_Sideline[i - 1] - Left_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Left_Sideline[i], i - 3, Left_Sideline[i - 3], 'k') - xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k')) > 0.5
//             && xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k') < 1
            //  && L_h_guai.flag == 0
            //  && i < L_l_guai.row
        )
        {
            L_h_guai.row = i + 1;
            L_h_guai.column = (uint8)Left_Sideline[i];
            L_h_guai.flag = 1;

//            if(R_h_guai.flag)
//                break;

        }

        /**************右上拐点**************/
        if(
             Right_Sideline[i] < LCDW_1 - 3 &&
             (
                   /*  0*
                       0010
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && !Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*   
                       0100
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && !Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                      01000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 1] && !Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
//                    ||/*    0*
//                          00010
//                     */
// (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0]
// && Image_Use[i + 1][Right_Sideline[i] + 1] && !Image_Use[i + 1][Right_Sideline[i] + 3]
// && Image_Use[i + 1][Right_Sideline[i] + 2])

             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Right_Sideline[i - 1] - Right_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
             && xielv_sideline(i, Right_Sideline[i], i + 1, Right_Sideline[i + 1], 'k') > -1
            //  && R_h_guai.flag == 0
            //  && i < R_l_guai.row
           )
        {
            R_h_guai.row = i + 1;
            R_h_guai.column = (uint8)Right_Sideline[i];
            R_h_guai.flag = 1;

//            if(L_h_guai.flag)
//                break;
        }

    }


//    else if(Mode_1)
//    {
        //找左下，右下拐点
        for(int i = imgInfo.bottom - 3; i >= imgInfo.top + 3 && Flag.Huandao_L != 3 && Flag.Huandao_R != 3; i--)
        {
            if(L_h_guai.flag == 0)
                L_h_guai.row = 1;
            if(R_h_guai.flag == 0)
                R_h_guai.row = 1;
            /**************左下拐点**************/
            if(
                 Left_Sideline[i] > 3 &&
                 (
                      /*  0000
                            *0
                                */
                     (Image_Use[i][Left_Sideline[i] +0] && Image_Use[i - 1][Left_Sideline[i] + 0]
                      && Image_Use[i - 1][Left_Sideline[i] - 1] && Image_Use[i - 1][Left_Sideline[i] - 2]  && Image_Use[i - 1][Left_Sideline[i] -3])
                     ||
                     /*  0100
                           *0
                               */
                     (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] - 1]
                      && !Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                      ||
                      /*  0010
                            *0
                                */
                      (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && !Image_Use[i - 1][Left_Sideline[i] - 1]
                       && Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] - white_width[i + 1] > 10
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i] && white_width[i - 4] > white_width[i]
                 && i > L_h_guai.row
                 && L_l_guai.flag == 0
            )
            {
                L_l_guai.row = i - 1;
                L_l_guai.column = (uint8)Left_Sideline[i];
                L_l_guai.flag = 1;

//                if(R_l_guai.flag)
//                    break;

            }

            /**************右下拐点**************/
            if(
                 Right_Sideline[i] < LCDW_1 - 3 &&
                 (
                         /*
                              0000
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0]
                         && Image_Use[i - 1][Right_Sideline[i] + 1] && Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                         ||
                         /*
                              0010
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                          && !Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                          ||
                          /*
                               0001
                               0*
                           */
                          (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                           && Image_Use[i - 1][Right_Sideline[i] + 2] && !Image_Use[i - 1][Right_Sideline[i] + 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i]
                 && white_width[i - 2] - white_width[i + 1] > 10
//                 && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
//                 && xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') > 1
                 && i > R_h_guai.row
                 && R_l_guai.flag == 0
            )
            {
                R_l_guai.row = i - 1;
                R_l_guai.column = (uint8)Right_Sideline[i];
                R_l_guai.flag = 1;

//                if(L_l_guai.flag)
//                    break;
            }
        }
    
//    }
int l_guai_max=0,r_guai_max=93;//,l_guai_line,r_guai_line
if(Flag.Huandao_R == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Left_Sideline[i]> l_guai_max)//
            {
                l_guai_max = Left_Sideline[i];
                L_l_guai.row = i ;
            }
    }

                L_l_guai.column = (uint8)Left_Sideline[L_l_guai.row]+3;
                L_l_guai.flag = 1;
}
if(Flag.Huandao_L == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Right_Sideline[i]< r_guai_max)//
            {
                r_guai_max = Right_Sideline[i];
                R_l_guai.row = i ;
            }
    }

                R_l_guai.column = (uint8)Right_Sideline[R_l_guai.row]-3;
                R_l_guai.flag = 1;
}
        // for(int i = imgInfo.bottom - 3; i >= imgInfo.top + 3 && Flag.Huandao_L != 3 && Flag.Huandao_R != 3; i--)



}


struct Guaidian  L_h_guai1,  R_h_guai1;  //拐点信息结构体

void Find_l_h_Guaidian(void)
{
    L_h_guai1.flag = 0;
    R_h_guai1.flag = 0;
            L_h_guai1.row = 0;
            // L_h_guai1.column =0;
    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {

        /**************左上拐点**************/
        if(
             Left_Sideline[i] > 3 &&
             (
                  /*    *0
                      0100
                           */
                 (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
                  && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0000
                           */
                  (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                   && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && !Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      00010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0]&& Image_Use[i + 1][Left_Sideline[i] + 1] && !Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
             )
             && white_width[i + 2] - white_width[i - 0] > 9
             && xielv_sideline(i, Left_Sideline[i], i + 1, Left_Sideline[i + 1], 'k') < 1
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Left_Sideline[i - 1] - Left_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Left_Sideline[i], i - 3, Left_Sideline[i - 3], 'k') - xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k')) > 0.5
//             && xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k') < 1
             && L_h_guai1.flag == 0
             && i < L_l_guai.row
        )
        {
            L_h_guai1.row = i + 1;
            L_h_guai1.column = (uint8)Left_Sideline[i];
            L_h_guai1.flag = 1;


        }


    }
}


void Find_r_h_Guaidian(void)
{
    R_h_guai1.flag = 0;
    L_h_guai1.flag = 0;
            R_h_guai1.row = 0;
            // R_h_guai1.column =0;
    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {
        /**************右上拐点**************/
        if(
             Right_Sideline[i] < LCDW_1 - 3 &&
             (
                   /*  0*
                       0010
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && !Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0100
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && !Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                      01000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 1] && !Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
             )
             && white_width[i + 2] - white_width[i - 0] > 9
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Right_Sideline[i - 1] - Right_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
             && xielv_sideline(i, Right_Sideline[i], i + 1, Right_Sideline[i + 1], 'k') > -1
             && R_h_guai1.flag == 0
             && i < R_l_guai.row
           )
        {
            R_h_guai1.row = i + 1;
            R_h_guai1.column = (uint8)Right_Sideline[i];
            R_h_guai1.flag = 1;

//            if(L_h_guai.flag)
//                break;
        }


    }
}

/***************************************************找中线**********************************************************/
void Find_Midline(){
//   if(imgInfo.L_loselineSum==0&&imgInfo.R_loselineSum==0){

    for(uint16 i = imgInfo.bottom-1;i>imgInfo.top;i--){
      Mid_Line[i] = (Left_Sideline[i] + Right_Sideline[i])/2;
    }
//   }
        for(uint16 i = imgInfo.bottom - 1; i > imgInfo.top; i--)
        {
            Last_Mid_Line[i] = Mid_Line[i];
        }
}


int top_white_num;
int right_num = 0,r_num,left_num = 0,l_num = 0,R_l_lsoe=0,L_l_lose=0;
int L_loseline_l ,L_loseline_h,R_loseline_l ,R_loseline_h;
float k1,kL,kR;
uint16_t maxkuan_line;
void straight_judge(void)
{
              if(R_h_guai.flag==1||L_h_guai.flag==1)//&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>25&&imgInfo.Both_lose==0 
{
                    Find_right_Sideline(imgInfo.bottom-5,imgInfo.top+1);
                    Find_left_Sideline(imgInfo.bottom-5,imgInfo.top+1); 
}


           maxkuan_line=15;
       uint16_t kuan_max = 0;
        for(int i = imgInfo.top;i < 52; i++)
        {
            if((uint16_t)white_width[i]> kuan_max&&white_width[i]<70)//
            {
                kuan_max = (uint16_t)white_width[i];
                maxkuan_line=(uint16_t)i;
            }
        }

        top_white_num=0;
    //    for(int i = imgInfo.top-2;i<imgInfo.top;i++)
    //    {
            for(int j = Left_Sideline[imgInfo.top + 2];j<Right_Sideline[imgInfo.top + 2];j++)
            {
                if(Image_Use[MAX(imgInfo.top-1,0)][j] == white)
                    top_white_num+=1;
            }
//        }
    int count=0;
    for(int i = imgInfo.bottom ;i>imgInfo.top+1;i--)
    {

        if(Left_Sideline_flag[i] == 0)
        {
            count++;
            if(count==1)
            L_loseline_h = i;
            L_loseline_l = i;
        }
        if(count==0)
        {
            L_loseline_h = 60;
            L_loseline_l = 60;
        }

    }
    count=0;
    for(int i = imgInfo.bottom ;i>imgInfo.top+1;i--)
    {

        if(Right_Sideline_flag[i] == 0)
        {
            count++;
            if(count==1)
            R_loseline_h = i;
            R_loseline_l = i;//最低点不更新
        }
        if(count==0)
        {
            R_loseline_h = 60;
            R_loseline_l = 60;
        }

    }
    r_num=0;
    float k = xielv_sideline(24, Right_Sideline[24], 34, Right_Sideline[34], 'k');
    float b = xielv_sideline(24, Right_Sideline[24], 34, Right_Sideline[34], 'b');

//    if(Flag.Huandao_R==5)
        kR=k;

        if(Flag.Huandao_R==6||Flag.Huandao_L==6)
        {
            for(int i = 18 ;i<45 ;i++)
            {
            if(abs(Right_Sideline[i]-k*i-b)>3)
            {
                r_num++;
            }
            }
        }

        else
        {
    for(int i = 12 ;i<36 ;i++)
    {   
    if(abs(Right_Sideline[i]-k*i-b)>3)
    {
        r_num++;
    }
    }
        }

    l_num=0;
     k = xielv_sideline(24, Left_Sideline[24], 34, Left_Sideline[34], 'k');
     b = xielv_sideline(24, Left_Sideline[24], 34, Left_Sideline[34], 'b');

//     if(Flag.Huandao_L==5)
     kL=k;

     if(Flag.Huandao_R==6||Flag.Huandao_L==6)
     {
         for(int i = 18;i<45 ;i++)
         {
             if(abs(Left_Sideline[i]-k*i-b)>3)
         {
             l_num++;
         }
         }
     }



     else
     {
    for(int i = 12 ;i<36 ;i++)
    {
        if(abs(Left_Sideline[i]-k*i-b)>3)
    {
        l_num++;
    }
    }
     }

    if(r_num<3)
    {
        imgInfo.R_straight_flag=1;
    }
    else
    {
        imgInfo.R_straight_flag=0;
    }
    if(l_num<3)
    {
        imgInfo.L_straight_flag=1;
    }
    else
    {
        imgInfo.L_straight_flag=0;
    }



       imgInfo.Both_lose=0;
       for(int i = imgInfo.top +5;i<=50 ;i++)
       {
           if(Left_Sideline_flag[i]==0&&Right_Sideline_flag[i]==0)
           {
                imgInfo.Both_lose++;
           }
       }
}


/***************************************************线性回归计算中线斜率**********************************************/
float B,A;
void regression(int startline,int endline)
{

  int i=0,SumX=0,SumY=0,SumLines = 0;
  float SumUp=0,SumDown=0,avrX=0,avrY=0;
  SumLines=endline-startline;   // startline 为开始行， //endline 结束行 //SumLines

  for(i=startline;i<endline;i++)
  {
    SumX+=Mid_Line[i];//列号（X坐标）
    SumY+=i;    //行号（Y坐标）
  }
  avrX=(float)SumX/SumLines;     //X平均值
  avrY=(float)SumY/SumLines;     //Y平均值
  SumUp=0;
  SumDown=0;
  for(i=startline;i<endline;i++)
  {
    SumUp+=(Mid_Line[i]-avrX)*(i-avrY);
    SumDown+=(i-avrY)*(i-avrY);
  }
  if(SumDown==0)
    B=0;
  else{
    B=(float)(SumUp/SumDown);
    A=avrX - B*avrY;  //截距
  }

}
/***************************************************曲率计算********************************************************/
float curvature;
void calculateCurvature(float x1,float y1,float x2,float y2,float x3,float y3){
  
  // 计算三角形的边长
  float a = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  float b = sqrt(pow(x3 - x2, 2) + pow(y3 - y2, 2));
  float c = sqrt(pow(x3 - x1, 2) + pow(y3 - y1, 2));

  // 计算三角形的半周长
  float s = (a + b + c) / 2;

  // 计算三角形的面积（使用海伦公式）
  float area = sqrt(s * (s - a) * (s - b) * (s - c));

  // 计算外接圆半径
  float radius = (a * b * c) / (4 * area);

  // 计算曲率（曲率是半径的倒数）
  curvature =(float) 1 / radius*100;  

}

float Yaw_Huandao,Yaw_Huandao_err,yaw_correct,distance_HUAN1;//1m=35000
float g_huandao_l_state5_exit_yaw = 25.0f;
float g_huandao_r_state5_exit_yaw = -55.0f;
// 仅用于日志标记函数入口状态；实际视觉逻辑保持作者式 Flag 同帧级联。
uint8_t g_huandao_L_geom = 0;
uint8_t g_huandao_R_geom = 0;

void Huandao_L_imu()
{
    // 日志快照；状态机和视觉行为均跟作者直接读取 Flag。
    g_huandao_L_geom = Flag.Huandao_L;

    if(Flag.Huandao_L >1)
    {
    huandao_yaw_correct();
    }
    if(Flag.Huandao_L == 0 && Flag.Huandao_R == 0&&Flag.picture!=2&&Flag.picture!=3&&Flag.picture!=4&&Flag.picture!=5)
    {

    if(imgInfo.top <= 12&&r_num<3&& imgInfo.L_loselineSum-imgInfo.R_loselineSum > 3&&R_l_guai.flag==0&&R_h_guai.flag==0&&L_l_guai.flag==1&&L_l_guai.row>20)//
    {
        distance_HUAN1=real_distance[L_l_guai.row]/100*66;
        Flag.Huandao_L = 1;

    }
    }
    /*左环岛*/
    /* 进入条件 左环岛标志位为0 右环岛标志位为0 左下拐点存在 右边丢线行数小于10行 左边丢线行数大于0行 右边没有拐点 截止行在图像较上面（前面不在弯道）*/
    /*此处受图像影响 待图像畸变较小时可以加上左上拐点的上面两行没有边线，便于识别*/


    if(Flag.Huandao_L == 1)
       {
           //进入左环岛判定后进行第二次左环岛判定，左边上下两个拐点都存在
           if(L_l_guai.flag == 1 && L_h_guai.flag == 1)
           {
               //要求左下拐点至少要在图像下方一点
               if(L_l_guai.row > LCDH_1 / 2)
               {
                   float k = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'k');
                   float b = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'b');


                   //从左下拐点遍历到左上拐点，通过斜率进行补线
                   for (int i = L_l_guai.row; i >= L_h_guai.row; i--)
                   {
                       if (!Image_Use[i][(int8)(k * i + b)])
                           continue;
                       else
                           Left_Sideline[i] = k * i + b;
                   }
                   Flag.Buxian = 1;
               }
               //若左下拐点在图像偏中上部分，则以左上拐点到图像下方第五列来进行斜率补线
               else if(L_l_guai.row <= LCDH_1 / 2)
               {
                   float k = xielv_sideline(L_h_guai.row, L_h_guai.column, LCDH_1 - 2, 5, 'k');
                   float b = xielv_sideline(L_h_guai.row, L_h_guai.column, LCDH_1 - 2, 5, 'b');


                   for (int i = LCDH_1 - 2; i >= L_h_guai.row; i--)
                   {

                       if(!Image_Use[i][(int8)(k * i + b)])
                           continue;
                       else
                           Left_Sideline[i] = k * i + b;
                   }
               }

           }
           //若进入了左环岛二级判断仍只有左下拐点
           else if(L_l_guai.flag == 1)
           {
               //从左下拐点到顶部边线进行补线
               float k = xielv_sideline(L_l_guai.row, L_l_guai.column, imgInfo.bottom - 1, Left_Sideline[imgInfo.bottom - 1], 'k');
               float b = xielv_sideline(L_l_guai.row, L_l_guai.column, imgInfo.bottom - 1, Left_Sideline[imgInfo.bottom - 1], 'b');

               for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
               {
                   if (!Image_Use[i][(int8)(k * i + b)])
                       continue;
                   else
                       Left_Sideline[i] = k * i + b;

               }
               Flag.Buxian = 1;
           }
           else if(R_l_guai.flag == 1 || R_h_guai.flag == 1 || imgInfo.R_loselineSum > 10)//
           {
               Flag.Huandao_L = 0;
           }
           //若圆环二级判断左上左下拐点都没有
           else
           {
               //顶部到底部进行补线
               float k = xielv_sideline(imgInfo.top + 1, Left_Sideline[imgInfo.top + 1], LCDH_1 - 2, 5, 'k');
               float b = xielv_sideline(imgInfo.top + 1, Left_Sideline[imgInfo.top + 1], LCDH_1 - 2, 5, 'b');

               for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
               {

                   if(!Image_Use[i][(int8)(k * i + b)])
                       continue;
                   else
                       Left_Sideline[i] = k * i + b;
               }
           }
           //若是左环岛二级判断底部三行左边线丢线了 且左边丢线行数大于25行，说明此时车身已经靠在圆环出口那里了
           if((Left_Sideline_flag[LCDH_1 - 4] == 0 && Left_Sideline_flag[LCDH_1 - 5] == 0) && imgInfo.L_loselineSum > 10&&distance>distance_HUAN1)
           {
               Flag.Huandao_L = 2;
               Yaw_Huandao=icm_data.yaw;
               distance=0;
               distance_HUAN1=0;
           }

       }

         if(Flag.Huandao_L == 2)
        {
              float k ;
              float b ;

              if( L_h_guai.flag == 1)
              {
              k = xielv_sideline(L_h_guai.row, L_h_guai.column, MIN(L_h_guai.row+5,57), Right_Sideline[MIN(L_h_guai.row+5,57)], 'k');
              b = xielv_sideline(L_h_guai.row, L_h_guai.column, MIN(L_h_guai.row+5,57), Right_Sideline[MIN(L_h_guai.row+5,57)], 'b');

             for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
             {
                 if(!Image_Use[i][(int8)(k * i + b)])
                    continue;
                 else
                 {
                     for(int m = 0; m <= 7; m++)
                     {
                         if(((k * i + b) + m) >= LCDW_1 - 1)
                         {
                             break;
                         }
                         else
                             Image_Use[i][(int8)(k * i + b) + m] = black;
                     }
                 }
             }

             Flag.Buxian = 1;
              }
            if( L_l_guai.flag == 1)//L_h_guai.flag == 1 &&
            {

                k=-0.5;
                b=L_l_guai.column-k*L_l_guai.row;
                for (int i = imgInfo.bottom-1; i >= L_l_guai.row; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                    {
                        for(int m = 0; m >=-7; m--)
                        {
                            if(((k * i + b) + m) <=0)
                            {
                                break;
                            }
                            else
                                Image_Use[i][(int8)(k * i + b) + m] = black;
                        }
                    }
                }



                Flag.Buxian = 1;

            }


            else
            {
                float k = xielv_sideline(3, 40, LCDH_1 - 2, 5, 'k');
                float b = xielv_sideline(3, 40, LCDH_1 - 2, 5, 'b');

                for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                        Left_Sideline[i] = k * i + b;
                }

                k = xielv_sideline(3 , 38, LCDH_1 - 1, LCDW_1 - 25, 'k');
                b = xielv_sideline(3 , 38, LCDH_1 - 1, LCDW_1 - 25, 'b');

                for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                        Right_Sideline[i] = k * i + b;
                }

                    Flag.Buxian = 1;
                }


            if(((Left_Sideline_flag[LCDH_1 - 2] == 1 && Left_Sideline_flag[LCDH_1 - 3] == 1 && Left_Sideline_flag[LCDH_1 - 4] == 1
                    && L_h_guai.flag) )&&distance>13.6)//&&distance>10|| !L_l_guai.flag
            {
                Flag.Huandao_L = 3;
                distance=0;
            }
            Get_ImageTop();
            Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);
        }

         if(Flag.Huandao_L == 3)
            {
             if( L_h_guai.flag == 1)//L_h_guai.flag == 1 &&
             {
                    float k = xielv_sideline(L_h_guai.row, L_h_guai.column, MIN(L_h_guai.row+20,57),Right_Sideline[MIN(L_h_guai.row+20,57)], 'k');
                    float b = xielv_sideline(L_h_guai.row, L_h_guai.column, MIN(L_h_guai.row+20,57),Right_Sideline[MIN(L_h_guai.row+20,57)], 'b');

                    for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
                    {
                        if(!Image_Use[i][(int8)(k * i + b)])
                           continue;
                        else
                        {
                            for(int m = 0; m <= 7; m++)
                            {
                                if(((k * i + b) + m) >= LCDW_1 - 1)
                                {
                                    break;
                                }
                                else
                                    Image_Use[i][(int8)(k * i + b) + m] = black;
                            }
                            Flag.Buxian = 1;
                        }
                    }
                }
            //  else if(  L_h_guai.flag == 0&&R_h_guai.flag == 1)//L_h_guai.flag == 1 &&
            //  {
            //         float k = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 1, LCDW_1 - 20, 'k');
            //         float b = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 1, LCDW_1 - 20, 'b');

            //         for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
            //         {
            //             if(!Image_Use[i][(int8)(k * i + b)])
            //                continue;
            //             else
            //             {
            //                 for(int m = 0; m <= 7; m++)
            //                 {
            //                     if(((k * i + b) + m) >= LCDW_1 - 1)
            //                     {
            //                         break;
            //                     }
            //                     else
            //                         Image_Use[i][(int8)(k * i + b) + m] = black;
            //                 }
            //                 Flag.Buxian = 1;
            //             }
            //         }
            //     }
//             else
//             {
//                 Eerr_flag=1;
////                    float k = xielv_sideline(1, 0, imgInfo.bottom - 1, LCDW_1 - 20, 'k');
////                    float b = xielv_sideline(1, 0, imgInfo.bottom - 1, LCDW_1 - 20, 'b');
////
////                    for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
////                    {
////                        if(!Image_Use[i][(int8)(k * i + b)])
////                           continue;
////                        else
////                        {
////                            for(int m = 0; m <= 7; m++)
////                            {
////                                if(((k * i + b) + m) >= LCDW_1 - 1)
////                                {
////                                    break;
////                                }
////                                else
////                                    Image_Use[i][(int8)(k * i + b) + m] = black;
////                            }
//                            Flag.Buxian = 1;
////                        }
////                    }
//                }
                //环岛3是直接把USE图像数组改了，所以重新找截止行来确定手动补线后的截止行和边线，这时找到的图像就是一个大弯拐进环岛
                Get_ImageTop();
                Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);

                //进入弯道
                if(Yaw_Huandao_err<-105)//(distance>30)&&imgInfo.Both_lose==0
                {
                    Flag.Huandao_L = 4;
//                    distance=0;
                }
            }

             if(Flag.Huandao_L == 4)
            {
                //找到右下拐点，即说明已经到了快要出弯的地方
                if(R_l_guai.flag)
                {
                    uint8 temp = 0;
                    //找到右赛道的最顶的边线，从那个点补到右下拐点上
//                    for(int m = imgInfo.bottom - 4; m > imgInfo.top + 3; m--)
//                    {
//                        if(!Left_Sideline_flag[m + 1] && !Left_Sideline_flag[m + 2] && !Left_Sideline_flag[m + 3] && !Left_Sideline_flag[m + 4] && !Left_Sideline_flag[m + 5] && Left_Sideline_flag[m - 1] && Left_Sideline_flag[m - 2])
//                        {
//                            temp =(uint8) m;
//                            break;
//                        }
//                    }
//                    if(temp == 0)
//                    {
                        temp = imgInfo.top + 1;
//                    }
                    float k = xielv_sideline(R_l_guai.row, R_l_guai.column, temp, 30, 'k');
                    float b = xielv_sideline(R_l_guai.row, R_l_guai.column, temp, 30, 'b');

                    for (int i = R_l_guai.row; i > imgInfo.top; i--)
                    {
                        for(int m = 0; m <= 13; m++)
                        {
                            if(((k * i + b) + m) >= LCDW_1 - 1)
                            {
                                break;
                            }
                            else
                            {
                                Image_Use[i][(uint8)(k * i + b) + m] = black;
                            }

                        }
                    }
                    Flag.Buxian = 1;
                }
            //    else if((!R_l_guai.flag) && abs(Left_Sideline[imgInfo.top+2]-Right_Sideline[imgInfo.top+2])>30 )
            //    {
            //        uint8 temp = 0;

            //         for(int m = imgInfo.bottom - 4; m > imgInfo.top + 1; m--)
            //         {
            //             if(!Left_Sideline_flag[m + 1] && !Left_Sideline_flag[m + 2] && !Left_Sideline_flag[m + 3] && !Left_Sideline_flag[m + 4] && !Left_Sideline_flag[m + 5] && Left_Sideline_flag[m - 1])
            //             {
            //                 temp =(uint8) m;
            //                 break;
            //             }
            //         }
            //         if(temp == 0)
            //         {
            //             temp = imgInfo.top + 1;
            //         }
            //        float k = xielv_sideline(LCDH_1 - 2, LCDW_1 - 2, temp, 30, 'k');
            //        float b = xielv_sideline(LCDH_1 - 2, LCDW_1 - 2, temp, 30, 'b');

            //        for (int i = LCDH_1 - 2; i >= temp - 2; i--)
            //        {
            //            for(int m = 0; m <= 4; m++)
            //            {
            //                if(((k * i + b) + m) >= LCDW_1 - 1)
            //                {
            //                    break;
            //                }
            //                else
            //                {
            //                    Image_Use[i][(uint8)(k * i + b) + m] = black;
            //                }

            //            }
            //        }
            //        Flag.Buxian = 1;
            //    }
                else
                {
                    Flag.Buxian = 1;
                }
                if(Yaw_Huandao_err<110&&Yaw_Huandao_err>100)//(L_h_guai.flag==1&&imgInfo.R_straight_flag==1)&&imgInfo.top<10
                {
                    Flag.Huandao_L = 5;
                }
                Get_ImageTop();
                Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);
                imgInfo.R_loselineSum = 0;
            }


              if(Flag.Huandao_L == 5)
                 {
                     uint8 temp = 0;
                     //找圆弧内白到黑的交界点
                     for(int m = imgInfo.bottom - 4; m > imgInfo.top + 3; m--)
                     {
                         if(!Left_Sideline_flag[m + 1] && !Left_Sideline_flag[m + 2] && !Left_Sideline_flag[m + 3]
                            && !Left_Sideline_flag[m + 4] && !Left_Sideline_flag[m + 5] && Left_Sideline_flag[m - 1])
                         {
                             temp = (uint8)m;
                             break;
                         }
                     }
                     if(temp == 0)
                     {
                         temp = imgInfo.top + 3;
                     }
                     float k = xielv_sideline(LCDH_1 - 2, LCDW_1 - 2, temp, 40, 'k');
                     float b = xielv_sideline(LCDH_1 - 2, LCDW_1 - 2, temp, 40, 'b');

                     //只要有右丢线情况就进行补线
                     if(imgInfo.R_loselineSum > 1)
                     {
                         for (int i = LCDH_1 - 2; i >= temp - 2; i--)
                         {
                             for(int m = 0; m <= 4; m++)
                             {
                                 if(((k * i + b) + m) >= LCDW_1 - 1)
                                 {
                                     break;
                                 }
                                 else
                                     Image_Use[i][(uint8)(k * i + b) + m] = black;
                             }
                         }
                         Flag.Buxian = 1;
                     }

                     //直到右边没有丢线情况并且左边出现拐点只后
                     if(Yaw_Huandao_err < g_huandao_l_state5_exit_yaw)//(L_h_guai.flag==1&&imgInfo.R_straight_flag==1)&&imgInfo.top<10
                     {
                         Flag.Huandao_L = 6;
                         Flag.Buxian = 0;
                     }
                     Get_ImageTop();
                     Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);
                 }

                  if(Flag.Huandao_L == 6)
                 {
                     if(L_h_guai.flag == 1)
                     {
                         //从左拐点向下补线，防止再次入环
                         float k = xielv_sideline(L_h_guai.row + 5, L_h_guai.column, LCDH_1 - 2, 8, 'k');
                         float b = xielv_sideline(L_h_guai.row + 5, L_h_guai.column, LCDH_1 - 2, 8, 'b');

                         for (int i = LCDH_1 - 2; i >= L_h_guai.row; i--)
                         {

                             if(!Image_Use[i][(uint8)(k * i + b)])
                                 continue;
                             else
                                 Left_Sideline[i] = k * i + b;
                         }
                     }
                     else if(L_h_guai.flag == 0)
                     {
                         float k = xielv_sideline(imgInfo.top + 10, Left_Sideline[imgInfo.top + 10], LCDH_1 - 2, 5, 'k');
                         float b = xielv_sideline(imgInfo.top + 10, Left_Sideline[imgInfo.top + 10], LCDH_1 - 2, 5, 'b');

                         for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                         {

                             if(!Image_Use[i][(uint8)(k * i + b)])
                                 continue;
                             else
                                 Left_Sideline[i] = k * i + b;
                         }
                     }
                     else
                         Flag.Buxian = 0;

                     // 初版 ae3379a：HL=6 退出仅 L_straight && distance>=10 && L_h_guai==0
                     // （勿加 Left_Sideline_flag[exit_row]，会把本可出环帧卡在 HL=6）
                     if(imgInfo.L_straight_flag==1&&distance>=10&&L_h_guai.flag==0)
                     {
                         Flag.Huandao_L = 0;
                         distance=0;
                     }
                 }

                  if((imgInfo.L_straight_flag==1&& imgInfo.R_straight_flag==1)&&L_h_guai.flag==0&&L_l_guai.flag==0&&R_h_guai.flag==0&&R_l_guai.flag==0&&abs(imgInfo.R_loselineSum-imgInfo.L_loselineSum)==0)//(Flag.Huandao_R>4&&  imgInfo.R_straight_flag==1)||
                  {
                                               Flag.Huandao_L = 0;
                  }
//                 if(Flag.Huandao_L && imgInfo.L_loselineSum < 10 && imgInfo.R_loselineSum < 10)
//                 {
//                     for(int i = imgInfo.bottom - 1; i > imgInfo.top + 1; i--)
//                     {
//                         if(i > imgInfo.top)
//                         {
//                             //右边界不发生突变且呈直线状态
//                             if(Right_Sideline[i] - Right_Sideline[i - 1] < 3 && Right_Sideline[i] >= Right_Sideline[i - 1])
//                             {
//                                 right_num ++;
//                             }
//                             //左边界不发生突变且呈直线状态
//                             if(Left_Sideline[i] - Left_Sideline[i - 1] < 3 && Left_Sideline[i] >= Left_Sideline[i - 1])
//                             {
//                                 left_num ++;
//                             }
//                         }
//                     }
//
//                     if(right_num > 50 && left_num > 50)
//                     {
//                         Flag.Huandao_L = 0;
//                         left_num = 0;
//                         right_num = 0;
//
//                     }
//                     else
//                     {
//                         left_num = 0;
//                         right_num = 0;
//                     }
//                 }


}










void Huandao_R_imu()
{
    // 日志快照；状态机和视觉行为均跟作者直接读取 Flag。
    g_huandao_R_geom = Flag.Huandao_R;

    if(Flag.Huandao_R >1)
    {
    huandao_yaw_correct();
    }
    if(Flag.Huandao_R == 0 && Flag.Huandao_L == 0&&Flag.picture!=2&&Flag.picture!=3&&Flag.picture!=4&&Flag.picture!=5)
    {

    if(imgInfo.top <= 12&&l_num<3&& imgInfo.R_loselineSum-imgInfo.L_loselineSum > 3&&L_l_guai.flag==0&&L_h_guai.flag==0&&R_l_guai.flag==1&&R_l_guai.row>20)//
    {
        distance_HUAN1=real_distance[R_l_guai.row]/100*66;
        Flag.Huandao_R = 1;

    }
    }
    /*右环岛*/
    /* 进入条件 左环岛标志位为0 右环岛标志位为0 左下拐点存在 右边丢线行数小于10行 左边丢线行数大于0行 右边没有拐点 截止行在图像较上面（前面不在弯道）*/
    /*此处受图像影响 待图像畸变较小时可以加上左上拐点的上面两行没有边线，便于识别*/


    if(Flag.Huandao_R == 1)
       {
           //进入左环岛判定后进行第二次左环岛判定，左边上下两个拐点都存在
           if(R_l_guai.flag == 1 && R_h_guai.flag == 1)
           {
               //要求左下拐点至少要在图像下方一点
               if(R_l_guai.row > LCDH_1 / 2)
               {
                   float k = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'k');
                   float b = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'b');


                   //从左下拐点遍历到左上拐点，通过斜率进行补线
                   for (int i = R_l_guai.row; i >= R_h_guai.row; i--)
                   {
                       if (!Image_Use[i][(int8)(k * i + b)])
                           continue;
                       else
                           Right_Sideline[i] = k * i + b;
                   }
                   Flag.Buxian = 1;
               }
               //若左下拐点在图像偏中上部分，则以左上拐点到图像下方第五列来进行斜率补线
               else if(R_l_guai.row <= LCDH_1 / 2)
               {
                   float k = xielv_sideline(R_h_guai.row, R_h_guai.column, LCDH_1 - 2, 88, 'k');
                   float b = xielv_sideline(R_h_guai.row, R_h_guai.column, LCDH_1 - 2, 88, 'b');


                   for (int i = LCDH_1 - 2; i >= R_h_guai.row; i--)
                   {

                       if(!Image_Use[i][(int8)(k * i + b)])
                           continue;
                       else
                           Right_Sideline[i] = k * i + b;
                   }
               }

           }
           //若进入了左环岛二级判断仍只有左下拐点
           else if(R_l_guai.flag == 1)
           {
               //从左下拐点到顶部边线进行补线
               float k = xielv_sideline(R_l_guai.row, R_l_guai.column, imgInfo.bottom - 1, Right_Sideline[imgInfo.bottom - 1], 'k');
               float b = xielv_sideline(R_l_guai.row, R_l_guai.column, imgInfo.bottom - 1, Right_Sideline[imgInfo.bottom - 1], 'b');

               for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
               {
                   if (!Image_Use[i][(int8)(k * i + b)])
                       continue;
                   else
                       Right_Sideline[i] = k * i + b;

               }
               Flag.Buxian = 1;
           }
           else if(L_l_guai.flag == 1 || L_h_guai.flag == 1 || imgInfo.L_loselineSum > 10)
           {
               Flag.Huandao_R = 0;
           }
           //若圆环二级判断左上左下拐点都没有
           else
           {
               //顶部到底部进行补线
               float k = xielv_sideline(imgInfo.top + 1, Right_Sideline[imgInfo.top + 1], LCDH_1 - 2, 88, 'k');
               float b = xielv_sideline(imgInfo.top + 1, Right_Sideline[imgInfo.top + 1], LCDH_1 - 2, 88, 'b');

               for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
               {

                   if(!Image_Use[i][(int8)(k * i + b)])
                       continue;
                   else
                       Right_Sideline[i] = k * i + b;
               }
           }
           //若是左环岛二级判断底部三行左边线丢线了 且左边丢线行数大于25行，说明此时车身已经靠在圆环出口那里了
           if((Right_Sideline_flag[LCDH_1 - 4] == 0 && Right_Sideline_flag[LCDH_1 - 5] == 0) && imgInfo.R_loselineSum > 10&&distance>distance_HUAN1)
           {
               distance=0;
               distance_HUAN1=0;
               Flag.Huandao_R = 2;
               Yaw_Huandao=icm_data.yaw;
           }
       }

         if(Flag.Huandao_R == 2)
        {
                float k ;
                float b ;

                if( R_h_guai.flag == 1)//L_h_guai.flag == 1 &&
                {
                    k = xielv_sideline(R_h_guai.row, R_h_guai.column, MIN(R_h_guai.row+5,57), Left_Sideline[MIN(R_h_guai.row+5,57)], 'k');
                    b = xielv_sideline(R_h_guai.row, R_h_guai.column,MIN(R_h_guai.row+5,57), Left_Sideline[MIN(R_h_guai.row+5,57)], 'b');

                   for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
                   {
                       if(!Image_Use[i][(int8)(k * i + b)])
                          continue;
                       else
                       {
                           for(int m = 0; m >=-7; m--)
                           {
                               if(((k * i + b) + m) <=0)
                               {
                                   break;
                               }
                               else
                                   Image_Use[i][(int8)(k * i + b) + m] = black;
                           }
                       }
                   }

                   Flag.Buxian = 1;
                }
            //左上拐点存在，继续补线
            if( R_l_guai.flag == 1)//L_h_guai.flag == 1 &&
            {

                k=0.5;
                b=R_l_guai.column-k*R_l_guai.row;
                for (int i = imgInfo.bottom-1; i >= R_l_guai.row; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                    {
                        for(int m = 0; m <=7; m++)
                        {
                            if(((k * i + b) + m) >=93)
                            {
                                break;
                            }
                            else
                                Image_Use[i][(int8)(k * i + b) + m] = black;
                        }
                    }
                }



                Flag.Buxian = 1;


            }

            else
            {
                float k = xielv_sideline(3, 53, LCDH_1 - 2, 88, 'k');
                float b = xielv_sideline(3, 53, LCDH_1 - 2, 88, 'b');

                for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                        Right_Sideline[i] = k * i + b;
                }

                k = xielv_sideline(3 , 55, LCDH_1 - 1, 24, 'k');
                b = xielv_sideline(3 , 55, LCDH_1 - 1, 24, 'b');

                for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                {

                    if(!Image_Use[i][(int8)(k * i + b)])
                        continue;
                    else
                        Left_Sideline[i] = k * i + b;
                }

                    Flag.Buxian = 1;
                }


            if(((Right_Sideline_flag[LCDH_1 - 2] == 1 && Right_Sideline_flag[LCDH_1 - 3] == 1 && Right_Sideline_flag[LCDH_1 - 4] == 1
                    && R_h_guai.flag) )&&distance>13.6)//|| !R_l_guai.flag
            {
                Flag.Huandao_R = 3;
                distance=0;
            }
            Get_ImageTop();
            Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);

        }

         if(Flag.Huandao_R == 3)
            {

             if( R_h_guai.flag == 1)//L_h_guai.flag == 1 &&
             {
                    float k = xielv_sideline(R_h_guai.row, R_h_guai.column, MIN(R_h_guai.row+20,57), Left_Sideline[MIN(R_h_guai.row+20,57)], 'k');
                    float b = xielv_sideline(R_h_guai.row, R_h_guai.column, MIN(R_h_guai.row+20,57), Left_Sideline[MIN(R_h_guai.row+20,57)], 'b');

                    for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
                    {
                        if(!Image_Use[i][(int8)(k * i + b)])
                           continue;
                        else
                        {
                            for(int m = 0; m >=-7; m--)
                            {
                                if(((k * i + b) + m) <=0)
                                {
                                    break;
                                }
                                else
                                    Image_Use[i][(int8)(k * i + b) + m] = black;
                            }
                            Flag.Buxian = 1;
                        }
                    }
                }
            //  else if(  R_h_guai.flag == 0&&L_h_guai.flag == 1)//L_h_guai.flag == 1 &&
            //  {
            //         float k = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 1,19, 'k');
            //         float b = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 1, 19, 'b');

            //         for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
            //         {
            //             if(!Image_Use[i][(int8)(k * i + b)])
            //                continue;
            //             else
            //             {
            //                 for(int m = 0; m >=- 7; m--)
            //                 {
            //                     if(((k * i + b) + m) <=0)
            //                     {
            //                         break;
            //                     }
            //                     else
            //                         Image_Use[i][(int8)(k * i + b) + m] = black;
            //                 }
            //                 Flag.Buxian = 1;
            //             }
            //         }   
            //     }

//             else
//             {
//                 Eerr_flag=1;
//
////                    float k = xielv_sideline(1, 0, imgInfo.bottom - 1, 19, 'k');
////                    float b = xielv_sideline(1, 0, imgInfo.bottom - 1, 19, 'b');
////
////                    for (int i = imgInfo.bottom - 1; i > imgInfo.top; i--)
////                    {
////                        if(!Image_Use[i][(int8)(k * i + b)])
////                           continue;
////                        else
////                        {
////                            for(int m = 0; m >= -7; m--)
////                            {
////                                if(((k * i + b) + m) <=0)
////                                {
////                                    break;
////                                }
////                                else
////                                    Image_Use[i][(int8)(k * i + b) + m] = black;
////                            }
//                            Flag.Buxian = 1;
//                        }
//                    }
//                }

                //环岛3是直接把USE图像数组改了，所以重新找截止行来确定手动补线后的截止行和边线，这时找到的图像就是一个大弯拐进环岛
                Get_ImageTop();
                Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);

                //进入弯道
                if(Yaw_Huandao_err>105)//(distance>30)&&imgInfo.Both_lose==0
                {
                   Flag.Huandao_R = 4;
//                    distance=0;
                }
            }

             if(Flag.Huandao_R == 4)
            {
                //找到右下拐点，即说明已经到了快要出弯的地方
                if(L_l_guai.flag)
                {
                    uint8 temp = 0;
                        temp = imgInfo.top + 1;
                    float k = xielv_sideline(L_l_guai.row, L_l_guai.column, temp, 63, 'k');
                    float b = xielv_sideline(L_l_guai.row, L_l_guai.column, temp, 63, 'b');

                    for (int i = L_l_guai.row; i > imgInfo.top; i--)
                    {
                        for(int m = 0; m >=-13; m--)
                        {
                            if(((k * i + b) + m) <= 0)
                            {
                                break;
                            }
                            else
                            {
                                Image_Use[i][(uint8)(k * i + b) + m] = black;
                            }

                        }
                    }
                    Flag.Buxian = 1;
                }
                else
                {
                                        Flag.Buxian = 1;
                }

                if(Yaw_Huandao_err>-110&&Yaw_Huandao_err<-100)//(L_h_guai.flag==1&&imgInfo.R_straight_flag==1)&&imgInfo.top<10
                {
                    Flag.Huandao_R = 5;
                }

                Get_ImageTop();
                Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);
                imgInfo.L_loselineSum = 0;
            }


              if(Flag.Huandao_R == 5)
                 {
                     uint8 temp = 0;
                     //找圆弧内白到黑的交界点
                     for(int m = imgInfo.bottom - 4; m > imgInfo.top + 3; m--)
                     {
                         if(!Right_Sideline_flag[m + 1] && !Right_Sideline_flag[m + 2] && !Right_Sideline_flag[m + 3]
                            && !Right_Sideline_flag[m + 4] && !Right_Sideline_flag[m + 5] && Right_Sideline_flag[m - 1])
                         {
                             temp = (uint8)m;
                             break;
                         }
                     }
                     if(temp == 0)
                     {
                         temp = imgInfo.top + 3;
                     }
                     float k = xielv_sideline(LCDH_1 - 2, 1, temp, 53, 'k');
                     float b = xielv_sideline(LCDH_1 - 2, 1, temp, 53, 'b');

                     //只要有右丢线情况就进行补线
                     if(imgInfo.L_loselineSum > 1)
                     {
                         for (int i = LCDH_1 - 2; i >= temp - 2; i--)
                         {
                             for(int m = 0; m >=-4; m--)
                             {
                                 if(((k * i + b) + m) <= 0)
                                 {
                                     break;
                                 }
                                 else
                                     Image_Use[i][(uint8)(k * i + b) + m] = black;
                             }
                         }
                         Flag.Buxian = 1;
                     }

                     //直到右边没有丢线情况并且左边出现拐点只后
                     if(Yaw_Huandao_err > g_huandao_r_state5_exit_yaw)//(R_h_guai.flag==1&&imgInfo.L_straight_flag==1)&&imgInfo.top<10
                     {
                         Flag.Huandao_R = 6;
                         Flag.Buxian = 0;
                     }
                     Get_ImageTop();
                     Find_Sideline(imgInfo.bottom - 1, imgInfo.top + 1);
                 }

                  if(Flag.Huandao_R == 6)
                 {
                     if(R_h_guai.flag == 1)
                     {
                         //从左拐点向下补线，防止再次入环
                         float k = xielv_sideline(R_h_guai.row + 5, R_h_guai.column, LCDH_1 - 2, 85, 'k');
                         float b = xielv_sideline(R_h_guai.row + 5, R_h_guai.column, LCDH_1 - 2, 85, 'b');

                         for (int i = LCDH_1 - 2; i >= R_h_guai.row; i--)
                         {

                             if(!Image_Use[i][(uint8)(k * i + b)])
                                 continue;
                             else
                                 Right_Sideline[i] = k * i + b;
                         }
                     }
                     else if(R_h_guai.flag == 0)
                     {
                         float k = xielv_sideline(imgInfo.top + 10, Right_Sideline[imgInfo.top + 10], LCDH_1 - 2, 88, 'k');
                         float b = xielv_sideline(imgInfo.top + 10, Right_Sideline[imgInfo.top + 10], LCDH_1 - 2, 88, 'b');

                         for (int i = LCDH_1 - 2; i >= imgInfo.top + 1; i--)
                         {

                             if(!Image_Use[i][(uint8)(k * i + b)])
                                 continue;
                             else
                                 Right_Sideline[i] = k * i + b;
                         }
                     }
                     else
                         Flag.Buxian = 0;

                     // 初版 ae3379a：HR=6 退出仅 R_straight && distance>=10 && R_h_guai==0
                     // （勿加 Right_Sideline_flag[exit_row]，与左环对称恢复）
                     if(imgInfo.R_straight_flag==1&&distance>=10&&R_h_guai.flag==0)
                     {
                         Flag.Huandao_R = 0;
                         distance=0;
                     }
                 }

                  if((imgInfo.L_straight_flag==1&& imgInfo.R_straight_flag==1)&&L_h_guai.flag==0&&L_l_guai.flag==0&&R_h_guai.flag==0&&R_l_guai.flag==0&&abs(imgInfo.R_loselineSum-imgInfo.L_loselineSum)==0)//(Flag.Huandao_R>4&&  imgInfo.R_straight_flag==1)||
                  {
                                               Flag.Huandao_R = 0;
                  }
//                 if(Flag.Huandao_R && imgInfo.R_loselineSum < 10 && imgInfo.L_loselineSum < 10)
//                 {
//                     for(int i = imgInfo.bottom - 1; i > imgInfo.top + 1; i--)
//                     {
//                         if(i > imgInfo.top)
//                         {
//                             //右边界不发生突变且呈直线状态
//                             if(Left_Sideline[i] - Left_Sideline[i - 1] < 3 && Left_Sideline[i] >= Left_Sideline[i - 1])
//                             {
//                                 right_num ++;
//                             }
//                             //左边界不发生突变且呈直线状态
//                             if(Right_Sideline[i] - Right_Sideline[i - 1] < 3 && Right_Sideline[i] >= Right_Sideline[i - 1])
//                             {
//                                 right_num ++;
//                             }
//                         }
//                     }
//
//                     if(left_num > 50 && right_num > 50)
//                     {
//                         Flag.Huandao_R = 0;
//                         left_num = 0;
//                         right_num = 0;
//
//                     }
//                     else
//                     {
//                         left_num = 0;
//                         right_num = 0;
//                     }
//                 }


}

/* 补线 */
void Buxian(void)
{

  
    Flag.Buxian = 0;
//    else if(Mode_1)
//    {
        //四个拐点都找到
        if(L_l_guai.flag && L_h_guai.flag && R_l_guai.flag && R_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)//
        {

            float kl, kr, bl, br;

            if(L_h_guai.row < L_l_guai.row)
            {
                kl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'k');
                bl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'b');

                for (int i = L_h_guai.row; i <= L_l_guai.row; i++)
                {
                    Left_Sideline[i] = kl * i + bl;
                    Left_Sideline_flag[i] = 1;
                }

                Flag.Buxian = 1;
            }

            if(R_h_guai.row < R_l_guai.row)
            {
                kr = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'k');
                br = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'b');

                for (int i = R_h_guai.row; i <= R_l_guai.row; i++)
                {
                    Right_Sideline[i] = kr * i + br;
                    Right_Sideline_flag[i] = 1;
                }

                Flag.Buxian = 1;
            }

//            Flag.Shizi=1;
        }

//        if(Flag.Shizi==1)
//        {
//            if(distance>30){Flag.Shizi=0;distance=0;}
        //找到左上，左下，右上
         if(L_l_guai.flag && L_h_guai.flag && R_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)//
        {
            float kl, kr, bl, br;

            if(L_h_guai.row < L_l_guai.row)
            {
                kl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'k');
                bl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'b');

                for (int i = L_h_guai.row; i <= L_l_guai.row; i++)
                {
                    Left_Sideline[i] = kl * i + bl;
                    Left_Sideline_flag[i] = 1;
                }

                Flag.Buxian = 1;
            }

            if(R_h_guai.row < L_l_guai.row)
            {
                kr = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5], 'k');
                br = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5], 'b');

                for (int i = R_h_guai.row; i <= imgInfo.bottom - 5; i++)
                {
                    Right_Sideline[i] = kr * i + br;
                    Right_Sideline_flag[i] = 1;
                    if (
                            (Right_Sideline[i] == Right_Sideline[i + 1] && Right_Sideline[i] == Right_Sideline[i + 2])
                            || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]
                        )
                        break;
                }

                Flag.Buxian = 1;
            }
        }


        else if(R_l_guai.flag && R_h_guai.flag && L_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)//
        {
            float kl, kr, bl, br;

            if(L_h_guai.row < R_l_guai.row)
            {
                kl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5], 'k');
                bl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5], 'b');

                for (int i = L_h_guai.row; i <= imgInfo.bottom - 5; i++)
                {
                    Left_Sideline[i] = kl * i + bl;
                    Left_Sideline_flag[i] = 1;
                    if(
                            (Left_Sideline[i] == Left_Sideline[i + 1] && Left_Sideline[i] == Left_Sideline[i + 2])
                            || !Image_Use[i + 1][(uint8)(kl * (i + 1) + bl)]
                        )
                        break;
                }

                Flag.Buxian = 1;
            }

            if(R_h_guai.row < R_l_guai.row)
            {
                kr = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'k');
                br = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'b');

                for (int i = R_h_guai.row; i <= R_l_guai.row; i++)
                {
                    Right_Sideline[i] = kr * i + br;
                    Right_Sideline_flag[i] = 1;
                }

                Flag.Buxian = 1;
            }

        }

        else if(R_h_guai.flag && L_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)//
        {
            float kl, kr, bl, br;

            kl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5] - 3, 'k');
            bl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5] - 3, 'b');

            for (int i = L_h_guai.row; i <= imgInfo.bottom - 5; i++)
            {
                Left_Sideline[i] = kl * i + bl;
                Left_Sideline_flag[i] = 1;
                if(
                        (Left_Sideline[i] == Left_Sideline[i + 1] && Left_Sideline[i] == Left_Sideline[i + 2])
                        || !Image_Use[i + 1][(uint8)(kl * (i + 1) + bl)]
                    )
                    break;
            }

            kr = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5] - 3, 'k');
            br = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5] - 3, 'b');

            for (int i = R_h_guai.row; i < imgInfo.bottom - 5; i++)
            {
                Right_Sideline[i] = kr * i + br;
                Right_Sideline_flag[i] = 1;
                if (
                        (Right_Sideline[i] == Right_Sideline[i + 1] && Right_Sideline[i] ==  Right_Sideline[i + 2])
                        || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]
                    )
                    break;
            }

            Flag.Buxian = 1;
        }

        else if(L_l_guai.flag && L_h_guai.flag)
        {
            float kl, bl;
            if(L_h_guai.row < L_l_guai.row)
            {
                kl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'k');
                bl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'b');

                for (int i = L_h_guai.row; i <= L_l_guai.row; i++)
                {
                    Left_Sideline[i] = kl * i + bl;
                    Left_Sideline_flag[i] = 1;
                }
            }

            Flag.Buxian = 1;
        }
        else if(R_l_guai.flag && R_h_guai.flag)
        {
            float kr, br;

            if(R_h_guai.row < R_l_guai.row)
            {
                kr = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'k');
                br = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'b');

                for (int i = R_h_guai.row; i <= R_l_guai.row; i++)
                {
                    Right_Sideline[i] = kr * i + br;
                    Right_Sideline_flag[i] = 1;
                }
            }

            Flag.Buxian = 1;
        }


       //只找到左上
       
       else if(L_h_guai.flag&&imgInfo.Both_lose>0)
       {
           float kl, bl;

           kl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 3, Left_Sideline[imgInfo.bottom - 3], 'k');
           bl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 3, Left_Sideline[imgInfo.bottom - 3], 'b');

           for (int i = L_h_guai.row; i <= imgInfo.bottom - 5; i++)
           {
               Left_Sideline[i] = kl * i + bl;
               Left_Sideline_flag[i] = 1;
               if(
                       (Left_Sideline[i] == Left_Sideline[i + 1] && Left_Sideline[i] == Left_Sideline[i + 2])
                       || !Image_Use[i + 1][(uint8)(kl * (i + 1) + bl)]
                  )
                  break;
            }

            Flag.Buxian = 1;
       }

       //只找到右上
       else if(R_h_guai.flag&&imgInfo.Both_lose>0)
       {
               float kr, br;

               kr = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 3, Right_Sideline[imgInfo.bottom - 3], 'k');
               br = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 3, Right_Sideline[imgInfo.bottom - 3], 'b');

               for (int i = R_h_guai.row; i < imgInfo.bottom - 5; i++)
               {
                   Right_Sideline[i] = kr * i + br;
                   Right_Sideline_flag[i] = 1;
                   if (
                           (Right_Sideline[i] == Right_Sideline[i + 1] && Right_Sideline[i] ==  Right_Sideline[i + 2])
                           || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]
                       )
                       break;
               }

               Flag.Buxian = 1;
       }

//        else if(L_l_guai.flag && R_l_guai.flag)
//        {
//            float kr, br;
//
//            kr = xielv_sideline(R_l_guai.row, R_l_guai.column, imgInfo.top + 5, Right_Sideline[imgInfo.top + 5], 'k');
//            br = xielv_sideline(R_l_guai.row, R_l_guai.column, imgInfo.top + 5, Right_Sideline[imgInfo.top + 5], 'b');
//
//            for (int i = R_l_guai.row; i > imgInfo.top + 5; i--)
//            {
//                Right_Sideline[i] = kr * i + br;
//                Right_Sideline_flag[i] = 1;
//                if (
//                        (Right_Sideline[i] == Right_Sideline[i + 1] && Right_Sideline[i] ==  Right_Sideline[i + 2])
//                        || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]
//                    )
//                    break;
//            }
//
//
//            kr = xielv_sideline(L_l_guai.row, L_l_guai.column, imgInfo.top + 5, Left_Sideline[imgInfo.top + 5], 'k');
//            br = xielv_sideline(L_l_guai.row, L_l_guai.column, imgInfo.top + 5, Left_Sideline[imgInfo.top + 5], 'b');
//
//            for (int i = L_l_guai.row; i > imgInfo.top + 5; i--)
//            {
//                Left_Sideline[i] = kr * i + br;
//                Left_Sideline_flag[i] = 1;
//                if (
//                        (Left_Sideline[i] == Left_Sideline[i + 1] && Left_Sideline[i] ==  Left_Sideline[i + 2])
//                        || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]
//                    )
//                    break;
//            }
//
//            Flag.Buxian = 1;
//        }

        else
            Flag.Buxian = 0;
//    }
//        else
//            Flag.Buxian = 0;
}


/***************************************************动态前瞻********************************************************/

int forward,forward1;
float g_forward_base = 30.0f;
float B_near,A_near,B_far,A_far,BA_ratio;
void dynamic_forward(){

regression(imgInfo.top+21,imgInfo.top+1);
B_far = B;
A_far = A;
regression(imgInfo.bottom-21,imgInfo.bottom-1);
B_near = B;
A_near = A;

regression(imgInfo.top + 1,imgInfo.bottom - 1);
//左弯
if(B<0){calculateCurvature((float)Left_Sideline[imgInfo.bottom - 1], (float)imgInfo.bottom - 1, (float)Left_Sideline[imgInfo.bottom - 11], (float)imgInfo.bottom - 11, (float)Left_Sideline[imgInfo.bottom - 21], (float)imgInfo.bottom - 21);}
//右弯
if(B>0){calculateCurvature((float)Right_Sideline[imgInfo.bottom - 1], (float)imgInfo.bottom - 1, (float)Right_Sideline[imgInfo.bottom - 11], (float)imgInfo.bottom - 11, (float)Right_Sideline[imgInfo.bottom - 21], (float)imgInfo.bottom - 21);}

float base_forward = 15;
// 根据曲率和近处斜率增大 forward（弯越急，看得越近）
float k_curv = 500.0f;  // 曲率增益
float k_slope = 80.0f;  // 斜率增益
float forward_candidate = base_forward  + k_curv * curvature  + k_slope * fabsf(B_near);
// 限幅到有效范围
if (forward_candidate < imgInfo.top+1) {
  forward = imgInfo.top+1;
} else if (forward_candidate > imgInfo.bottom-1) {
  forward = imgInfo.bottom-1;
  } 
else {
  forward = (int)forward_candidate;
  }
  // 极端急弯强制看最近
if (curvature > 0.12f || fabsf(B_near) > 0.8f) {
  forward = imgInfo.bottom-1;
  }
}

/***************************************************误差计算********************************************************/
float Dir_err = 0, Last_Dir_err = 0,Dir_Err[60],D_ERR;  //图像误差
void Err_Sum(void)
{




forward=forward1;
for(int i=imgInfo.top + 1;i<imgInfo.bottom - 1;i++)
{
    Dir_Err[i]=(float)(LCDW_1/2-(((float)Left_Sideline[i]+(float)Right_Sideline[i])/2.0f));
    //  if(!( Dir_Err[i]<100&& Dir_Err[i]>-100)) Dir_Err[i]=0;
}
    BA_ratio=0;
    for(int i=imgInfo.top + 1;i<imgInfo.bottom - 1;i++)
    {
        Dir_Err[i] = (1-BA_ratio)*Dir_Err[i] + BA_ratio*(B*i + A);
    }

        //  if(Flag.Huandao_R==2||Flag.Huandao_L==2)forward=-1;
        //  if(Flag.Huandao_R==3||Flag.Huandao_L==3)forward=-1;
         // ae3379a：仅状态 5 时 forward+=1（行号增大=看近）
         if(Flag.Huandao_R==5||Flag.Huandao_L==5)   forward+=1;

     if(forward<imgInfo.top+1)forward=imgInfo.top+1;
     if(forward>50)forward=50;
     
        Dir_err=Dir_Err[forward];

    // if(Flag.small_rock==1)Dir_err-=10;
    // if(Flag.small_rock==2)Dir_err+=10;

    // if(Flag.Zebra_cross==1)Dir_err=Dir_Err[maxkuan_line]*90/(120-maxkuan_line);
    // if(Flag.Zebra_cross==2)Dir_err=Dir_Err[maxkuan_line]*90/(120-maxkuan_line);
    // if(Flag.Zebra_cross==4)Dir_err=Dir_Err[maxkuan_line]*90/(120-maxkuan_line);

        if(Flag.picture==3)
            Dir_err = (float)(LCDW_1/2 - (Left_Sideline[forward] - g_picture_side_offset));
        if(Flag.picture==4)
            Dir_err = (float)(LCDW_1/2 - (Right_Sideline[forward] + g_picture_side_offset));

    if (g_run_mode == RUN_MODE_TRACK)
    {
        const red_event::BrickDetourRuntimeSnapshot detour_snapshot =
            red_event::LoadBrickDetourState();
        const red_event::BrickDetourCommand detour_command =
            red_event::RedBrickDetour::CommandFor(
                detour_snapshot.state, detour_snapshot.direction,
                Param_RedBrickDetourConfig());
        Dir_err = red_event::ApplyBrickDetourToImageError(Dir_err, detour_command);
    }

    if(!(Dir_err<100&&Dir_err>-100))Dir_err=0;
    if(Dir_err>47)Dir_err=47;    
    if(Dir_err<-47)Dir_err=-47;

            if(Flag.small_rock ==1)
    {
        Dir_err-=10;
    }

    if(Flag.small_rock ==2)
    {
        Dir_err+=10;
    }
    // if(Dir_err<-47)Dir_err=-47;
    // if(Dir_err>47)Dir_err=47;


                    D_ERR=Dir_err-Last_Dir_err;

                // if(Flag.picture==2)
                // {
                //                     if((D_ERR)>1)Dir_err=Last_Dir_err+4;
                // if((D_ERR)<-1)Dir_err=Last_Dir_err-4;
                // }
                // else{
                // if(Flag.picture==0)
                // {

                if((D_ERR)>=4)Dir_err=Last_Dir_err+4.0f;
                else if((D_ERR)<-4)Dir_err=Last_Dir_err-4.0f;
            //     // }
            //     // }
            Last_Dir_err = Dir_err;

        // Image.Kp = 3.5*3.0;//  0.8; //250*0.015
        // if(Image.Kp>4)Image.Kp=4;

        // if(Now_Speed<200)Image.Kp=3.5*2;        
        // if(Now_Speed<150)Image.Kp=3.5*Now_Speed*0.007;
        // if(fabs(Dir_err)<10)Image.Kp=3.5*fabs(Dir_err)*0.3;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/1)
        // {
        Image.Kp=3.5*Master_Speed/400*3.0;
        // if(Image.Kp<3.5*0.5)Image.Kp=3.5*0.5;
        // Image.Kd=Image.Kp*0.3;
        // Dis_1.Kp=Image.Kp/3.5/3;
        //  Dis_1.Kp = Master_Speed/400*60;//
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/1.5)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/400*2.5;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/2)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/400*1.5;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/3)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/400*1;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/4)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/400*0.5;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/5)Image.Kp=0;
        // if(fabs(Dir_err)<15)Image.Kp*=0.75;
        // else if(fabs(Dir_err)<5)Image.Kp*=0.5;
        // else if(fabs(Dir_err)<5)Image.Kp*=0.25;
        // }
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/2)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/speed_goal*3;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/3)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/speed_goal*2;
        // if(MAX(encoder_L.speed,encoder_R.speed)<speed_goal/4)Image.Kp=3.5*MAX(encoder_L.speed,encoder_R.speed)/speed_goal*1;
        

        if(real_distance[imgInfo.top]>150)Image.Kp=3.5*Master_Speed/400*2.75;
        // // if(Now_Speed<0)Image.Kp=0; 
        if(real_distance[imgInfo.top]>200)Image.Kp=3.5*Master_Speed/400*2.5;
        if(real_distance[imgInfo.top]>250)Image.Kp=3.5*Master_Speed/400*2.0;
        if(real_distance[imgInfo.top]>300)Image.Kp=3.5*Master_Speed/400*1.5;
        // if(Flag.picture==2)Image.Kp=3.5*1.5;

        // 标定旋钮：作者动态表之后只乘一次（flash 不再叠乘）
        if (g_image_kp_scale > 0.0f)
            Image.Kp *= g_image_kp_scale;

        Image_out =Image_PID_Calculate(&Image,Dir_err,0);//-icm_data.gyro_z//Image_E2
}



int red_find_x,red_find_y,red_find_y1;
int picture_first_num,picture_second_num,picture_third_num,maxlong_colume,colume_long[94], long_max,picture_white=0,picture_black=0,jump_point1;
float black_ratio;
// int red_find_x,red_find_y,red_find_y1;
// int picture_first_num,picture_second_num,picture_third_num,maxlong_colume,colume_long[94], long_max,picture_white=0,picture_black=0,jump_point1;
// float black_ratio;
int red_x_mid,red_y_mid,x_err_red;
float err_picture;
int red_left,red_right;
float real_picture_distance,recognize_distance,recognize_distance2;
void picture(void)
{
        if (!IsRedTargetProcessingEnabled())
        {
            Flag.picture = 0;
            Flag.Redblock = 0;
            Flag.infer = 0;
            Flag.supply = 0;
            Flag.vehicle = 0;
            Flag.weapon = 0;
            distance_picture = 0;
            red_objects.clear();
            return;
        }

        picture_white=0;
        picture_black=0;
        jump_point1=0;
        black_ratio=0;
        err_picture=100;
        x_err_red=200;



                   float K1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'k');
                   float B1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'b');
                    float K2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'k');
                   float B2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'b');
                   red_left=K1*MAX(R_h_guai.row,L_h_guai.row)+B1;
                   red_right=K2*MAX(R_h_guai.row,L_h_guai.row)+B2;

                recognize_distance = Now_Speed * g_picture_recog_k + g_picture_recog_base;
                recognize_distance2 = g_picture_near_dist;
    if(Flag.picture==0)
{

      if(R_h_guai.flag==1&&L_h_guai.flag==1)//&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>25&&imgInfo.Both_lose==0 
{
 
 //           &&(real_distance[imgInfo.top]-real_distance[MAX(R_h_guai.row,L_h_guai.row)])>60&&real_distance[imgInfo.top]>100&&MAX(R_h_guai.row,L_h_guai.row)<real_distance_to_row(40)
            if(MAX(R_h_guai.row,L_h_guai.row)>real_distance_to_row(recognize_distance))
            {
                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[MAX(R_h_guai.row,L_h_guai.row)]+12);//red_find_y到R_h_guai.row（*4)
                              if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
               red_find_y1=MAX(R_h_guai.row,L_h_guai.row);
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right- red_find_x,red_find_y1 - red_find_y);
                // cv::Rect rect(red_find_x,red_find_y,Right_Sideline[MAX(R_h_guai.row,L_h_guai.row)],red_find_y1 - red_find_y);
                // cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);
        
                //cv::circle(resizedFrame, bestCenter, 1, cv::Scalar(0, 255, 0), -1);
            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[MAX(R_h_guai.row,L_h_guai.row)]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<20)//&&abs(center.x-R_h_guai.column)<20)
                Flag.Redblock=1;
               }
            } 


                
}




     else if(R_h_guai.flag==1){
        //&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>60&&real_distance[imgInfo.top]>100&&R_h_guai.row<real_distance_to_row(40)
      if(R_h_guai.row>real_distance_to_row(recognize_distance)
      )
{


                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[R_h_guai.row]+12);//red_find_y到R_h_guai.row（*4)
               if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
               red_find_y1=R_h_guai.row;
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
                // cv::Rect rect(red_find_x,red_find_y,Right_Sideline[R_h_guai.row],red_find_y1 - red_find_y);
                // cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);
        
                //cv::circle(resizedFrame, bestCenter, 1, cv::Scalar(0, 255, 0), -1);
                             
            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[R_h_guai.row]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<15)//&&abs(center.x-R_h_guai.column)<20
                Flag.Redblock=1;
               }


}
}

      else if(L_h_guai.flag==1){
        //&&(real_distance[imgInfo.top]-real_distance[L_h_guai.row])>60&&real_distance[imgInfo.top]>100&&L_h_guai.row<real_distance_to_row(40)
       if(L_h_guai.row>real_distance_to_row(recognize_distance)
      )
{



                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//Right_Sideline[L_h_guai.row]
               red_find_y=real_distance_to_row(real_distance[L_h_guai.row]+12);//
                              if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
                            red_find_y1=L_h_guai.row;
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
            //    cv::Rect rect(red_find_x,red_find_y,Right_Sideline[L_h_guai.row],red_find_y1 - red_find_y);
            //    cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);

                            
            if(!red_objects.empty())
               { 
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[L_h_guai.row]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<15)//&&abs(center.x-L_h_guai.column)<20
                Flag.Redblock=1;
               }

}}






               if(Flag.Redblock==1){Flag.picture=2;}
    //         if(x_err_red>5&&imgInfo.R_straight_flag==0&&imgInfo.R_straight_flag==1)
    //         {
    //            Flag.small_rock =1;
    //         }

    //         if(x_err_red>5&&imgInfo.R_straight_flag==1&&imgInfo.R_straight_flag==0)
    //         {
    //            Flag.small_rock =2;
    //         }

    // if(Flag.small_rock ==1||Flag.small_rock ==2)
    // {
    //     if(distance>10)
    //     {
    //         distance=0;
    //         Flag.small_rock =0;
    //     }
    // }

}





    if(Flag.picture==1){
        
    }
    if(Flag.picture==2){

        Flag.infer = 1;
        if((L_h_guai.flag==1&&real_distance[MAX(R_h_guai.row,L_h_guai.row)]<g_picture_near_dist)||(R_h_guai.flag==1&&abs(recognize_distance2-real_distance[MAX(R_h_guai.row,L_h_guai.row)])<2))//&&fabs(Now_Speed)<10
        {
    //    Image.Kp/=2.5;
           if(R_h_guai.flag==1&&L_h_guai.flag==1)//&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>25&&imgInfo.Both_lose==0 
            {
 
            // if(MAX(R_h_guai.row,L_h_guai.row)>real_distance_to_row(80)&&(real_distance[imgInfo.top]-real_distance[MAX(R_h_guai.row,L_h_guai.row)])>60&&real_distance[imgInfo.top]>100) 
            // {
                Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[MAX(R_h_guai.row,L_h_guai.row)]+12);//red_find_y到R_h_guai.row（*4)
                              if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
               red_find_y1=MAX(R_h_guai.row,L_h_guai.row);
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
                // cv::Rect rect(red_find_x,red_find_y,Right_Sideline[MAX(R_h_guai.row,L_h_guai.row)],red_find_y1 - red_find_y);
                // cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);
        
                //cv::circle(resizedFrame, bestCenter, 1, cv::Scalar(0, 255, 0), -1);
            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[MAX(R_h_guai.row,L_h_guai.row)]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<20)//&&abs(center.x-R_h_guai.column)<20)
                // Flag.Redblock=1;
               }
            // } 


                
}





      else if(R_h_guai.flag==1&&abs(recognize_distance2-real_distance[R_h_guai.row])<2)//&&fabs(Now_Speed)<10&&R_h_guai.row>real_distance_to_row(80)&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>60&&real_distance[imgInfo.top]>100
{


                Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[R_h_guai.row]+12);//red_find_y到R_h_guai.row（*4)
                              if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
               red_find_y1=R_h_guai.row;
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
                // cv::Rect rect(red_find_x,red_find_y,Right_Sideline[R_h_guai.row],red_find_y1 - red_find_y);
                // cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);
        
                //cv::circle(resizedFrame, bestCenter, 1, cv::Scalar(0, 255, 0), -1);
                             
            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[R_h_guai.row]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<15)//&&abs(center.x-R_h_guai.column)<20
                // Flag.Redblock=1;
               }


}

      else if(L_h_guai.flag==1&&abs(recognize_distance2-real_distance[L_h_guai.row])<2)//&&fabs(Now_Speed)<10&&L_h_guai.row>real_distance_to_row(80)&&(real_distance[imgInfo.top]-real_distance[L_h_guai.row])>60&&real_distance[imgInfo.top]>100
{



                Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//Right_Sideline[L_h_guai.row]
               red_find_y=real_distance_to_row(real_distance[L_h_guai.row]+12);//
                              if(red_find_y<imgInfo.top)red_find_y=imgInfo.top;
                            red_find_y1=L_h_guai.row;
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
            //    cv::Rect rect(red_find_x,red_find_y,Right_Sideline[L_h_guai.row],red_find_y1 - red_find_y);
            //    cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);

                            
            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[L_h_guai.row]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<15)//&&abs(center.x-L_h_guai.column)<20
                // Flag.Redblock=1;
               }
}

                if (Flag.weapon >= 1)
                {
                    printf("检测到 weapon，左绕\n");
                    Flag.infer = 0;
                    Flag.supply = 0;
                    Flag.vehicle = 0;
                    Flag.weapon = 0;
                    Flag.picture = 3;
                    if (g_run_mode == RUN_MODE_TRACK &&
                        !race_recognition_result::Record(
                            race_recognition_result::RecognitionClass::kWeapon))
                        printf("[RACE_RESULT] weapon event write failed\n");
                }
                else if (Flag.supply >= 1)
                {
                    printf("检测到 supplies，右绕\n");
                    Flag.infer = 0;
                    Flag.supply = 0;
                    Flag.vehicle = 0;
                    Flag.weapon = 0;
                    Flag.picture = 4;
                    if (g_run_mode == RUN_MODE_TRACK &&
                        !race_recognition_result::Record(
                            race_recognition_result::RecognitionClass::kSupplies))
                        printf("[RACE_RESULT] supplies event write failed\n");
                }
                else if (Flag.vehicle >= 1)
                {
                    printf("检测到 vehicle，直行\n");
                    Flag.infer = 0;
                    Flag.supply = 0;
                    Flag.vehicle = 0;
                    Flag.weapon = 0;
                    Flag.picture = 5;
                    if (g_run_mode == RUN_MODE_TRACK &&
                        !race_recognition_result::Record(
                            race_recognition_result::RecognitionClass::kVehicle))
                        printf("[RACE_RESULT] vehicle event write failed\n");
                }



        }
    }

    if(Flag.picture==3){
        if(distance_picture>g_picture_hold_dist)
        {
            distance_picture=0;
            Flag.picture = 6;
            printf("左绕结束\n");
        }
    }

    if(Flag.picture==4){
        if(distance_picture>g_picture_hold_dist)
        {
            distance_picture=0;
            Flag.picture = 6;
            printf("右绕结束\n");
        }
    }

    if(Flag.picture==5){
       
        if(distance_picture>g_picture_hold_dist)
        {
            distance_picture=0;
            Flag.picture = 6;
            distance_picture=0;
            printf("直行阶段结束\n");

        }
    }

            if(Flag.picture==6){
        if(distance_picture>g_picture_finish_dist)
        {
            distance_picture=0;
            // Image.Kp=3.5*3.0;
            //结束绕行
            Flag.picture = 0;//清除标志位
            distance_picture=0;
            printf("picture结束\n");

        }
            }

}



void protect(void)
{
// if(Flag.Zebra_cross != 1&&Flag.Zebra_cross != 4&&run_flag==1)//&&Flash.mtv_exposure_time>=100
// {
//     if(imgInfo.top >=57||fabs(icm_data.gyro_z)>30)//&&Flag.Zhangai != 1
//     {
//         run_flag =2;
//         printf("异常触发保护\r\n");
//     }//||Speed_Encoder_l>2000||Speed_Encoder_r>2000||Speed_now>1500


// }
}
int g_camera_ok = 0;
uint32_t g_image_frame_id = 0;
float g_image_age_ms = 1.0e9f;
static std::chrono::steady_clock::time_point s_image_last_tp;
static int s_image_has_frame = 0;

float Image_AgeMs_Now(void)
{
    if (!s_image_has_frame)
        return 1.0e9f;
    const auto now = std::chrono::steady_clock::now();
    return (float)std::chrono::duration<double, std::milli>(now - s_image_last_tp).count();
}

static void image_mark_success_frame(void)
{
    g_image_frame_id++;
    s_image_last_tp = std::chrono::steady_clock::now();
    s_image_has_frame = 1;
    g_image_age_ms = 0.0f;
}

int image_init(void)
{
    g_camera_ok = 0;
    s_image_has_frame = 0;
    g_image_frame_id = 0;
    g_image_age_ms = 1.0e9f;
    s_red_event_router.Configure(Param_RedEventRouterConfig());
    s_red_brick_detour.Configure(Param_RedBrickDetourConfig());

    if (g_run_mode == RUN_MODE_TRACK &&
        !race_recognition_result::Reset(LQ_NCNN::ModelId()))
    {
        printf("[RACE_RESULT] reset failed; control continues without race result file\n");
    }

    // 模型文件与主程序同放 /root；失败时不启动视觉链，也不回退 TFLite。
    if (!classifier.Init(LQ_NCNN::ParamFileName(), LQ_NCNN::BinFileName()))
    {
        printf("[REC] NCNN init failed: %s (cwd must contain %s and %s)\n",
               classifier.LastError(),
               LQ_NCNN::ParamFileName(), LQ_NCNN::BinFileName());
        return -1;
    }
    printf("[REC] NCNN identity expected_param_sha256=%s expected_bin_sha256=%s\n",
           LQ_NCNN::ExpectedParamSha256(), LQ_NCNN::ExpectedBinSha256());
    constexpr uint16 kCameraWidth = 320;
    constexpr uint16 kCameraHeight = 240;
    constexpr uint16 kCameraFps = 60;
    if (uvc_dev.init(UVC_PATH, kCameraWidth, kCameraHeight, kCameraFps) == 0 &&
        uvc_dev.is_camera_opened())
    {
        printf("Camera opened successfully! backend=zf_device_uvc source=%dx%d YUYV@%d\n",
               kCameraWidth, kCameraHeight, kCameraFps);
        g_camera_ok = 1;
    }
    else{
        printf("Camera opened failed!\n");
        g_camera_ok = 0;
        return -1;
    }
    if (Flash.camera_exposure < 0)
    {
        printf("[camera] exposure=driver-default mode=%.0f current=%.0f (no app override)\n",
               uvc_dev.get_auto_exposure_mode(), uvc_dev.get_current_exposure());
    }
    else if (Flash.camera_exposure == 0)
    {
        if (uvc_dev.set_auto_exposure(UVC_AUTO_EXPOSURE_ENABLE) != 0)
        {
            printf("[camera] auto exposure rejected\n");
            g_camera_ok = 0;
            return -1;
        }
        printf("[camera] exposure=auto mode=%.0f current=%.0f\n",
               uvc_dev.get_auto_exposure_mode(), uvc_dev.get_current_exposure());
    }
    else if (uvc_dev.set_auto_exposure(UVC_AUTO_EXPOSURE_DISABLE) != 0 ||
             uvc_dev.set_exposure_value(Flash.camera_exposure) != 0)
    {
        printf("[camera] manual exposure=%d rejected\n", Flash.camera_exposure);
        g_camera_ok = 0;
        return -1;
    }
    else
    {
        printf("[camera] exposure=manual value=%d mode=%.0f current=%.0f\n",
               Flash.camera_exposure, uvc_dev.get_auto_exposure_mode(),
               uvc_dev.get_current_exposure());
    }
    //预分配内存
    grayFrame.create(LCDH_1, LCDW_1, CV_8UC1);
    // 必须 BGR：DetectRedBlock 按 Vec3b 读色；灰度会导致永远检不到红
    resizedFrame.create(LCDH_1, LCDW_1, CV_8UC3);
    translatedFrame.create(LCDH_1, LCDW_1, CV_8UC1);
    binaryFrame.create(LCDH_1, LCDW_1, CV_8UC1);
    translationMatrix = (cv::Mat_<float>(2, 3) << 1, 0, 0, 0, 1, 0);//将捕获图像向右平移3个像素点
   camera_server.start_server(8080);//打开图传服务器
    return 0;
}


uint16_t jump_point,finish_flag;
float distance_cross,distance,distance_picture;


void distance_judge(void) // 1m≈66 legacy units（ae3379a）
{
    // 元素 FSM 专用 legacy 里程：avg_count/350。与物理 cpm/cm 分账，阈值不随单位迁移改写。
    // 覆盖：圆环 1/2/6、斑马线 1/2/4、坡道、小路障；picture 用 distance_picture。
    const float avg_cnt =
        0.5f * ((float)encoder_L.count_now + (float)encoder_R.count_now);
    const float d_legacy = avg_cnt / 350.0f;
    if (Flag.Huandao_L == 1 || Flag.Huandao_R == 1 || Flag.Huandao_L == 2 || Flag.Huandao_R == 2 ||
        Flag.Huandao_R == 6 || Flag.Huandao_L == 6 || Flag.small_rock || Flag.Zebra_cross == 1 ||
        Flag.Zebra_cross == 2 || Flag.Zebra_cross == 4 || Flag.ramp != 0)
        distance += d_legacy;
    if (Flag.picture == 3 || Flag.picture == 4 || Flag.picture == 5 || Flag.picture == 6)
        distance_picture += d_legacy;
}


 void zebra_corssing(void)
 {


    if(Flag.Zebra_cross==0)
    {
    jump_point=0;
    for(int i = 25;i < 44; i++)
    {
        for(int j = 30; j < 64; j++)
        {
            if(Image_Use[i][j] == 255 && Image_Use[i][j+1] == 0)
            {
                jump_point ++;
            }
        }
    }

    if(jump_point>25)//&&top_white_num>0&&fabs(real_distance[imgInfo.top]-real_distance[MAX(R_h_guai.row,L_h_guai.row)])<20&&imgInfo.top>30
    {
                Flag.Zebra_cross=1;
            // }

    }
    }

    if(Flag.Zebra_cross==1)
    {

        if(distance>15)
        {
            Flag.Zebra_cross=2;
            distance=0;
        }
    }

    if(Flag.Zebra_cross==2)
    {
        if(distance>30)
        {
            Flag.Zebra_cross=3;
            distance=0;
        }
    }

    if(Flag.Zebra_cross==3)
    {
        jump_point=0;
        for(int i = 25;i < 44; i++)
        {
            for(int j = 30; j < 64; j++)
            {
                if(Image_Use[i][j] == 255 && Image_Use[i][j+1] == 0)
                {
                    jump_point ++;
                }
            }
        }


        if(jump_point>25&&Flag.ramp==0&&Flag.picture==0)//&&top_white_num>0&&fabs(real_distance[imgInfo.top]-real_distance[MAX(R_h_guai.row,L_h_guai.row)])<20&&imgInfo.top>30
        {
            Flag.Zebra_cross=4;
        }
    }

    if(Flag.Zebra_cross==4)
    {
        if(distance>10)
        {
            Flag.Zebra_cross=5;
            run_flag=2;
            if (g_run_mode == RUN_MODE_TRACK &&
                !race_recognition_result::Finalize())
            {
                printf("[RACE_RESULT] finalize failed at second zebra\n");
            }
        }

    }

 }






uint16_t small_rock_r_num,small_rock_l_num;
void small_rock(void)
{
    if(Flag.small_rock ==0)
    {
                   float K1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'k');
                   float B1 = xielv_sideline(55, Left_Sideline[55], 45, Left_Sideline[45], 'b');
                    float K2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'k');
                   float B2 = xielv_sideline(55, Right_Sideline[55], 45, Right_Sideline[45], 'b');
                   red_left=K1*MAX(R_h_guai.row,L_h_guai.row)+B1;
                   red_right=K2*MAX(R_h_guai.row,L_h_guai.row)+B2;


      if(R_h_guai.flag==1&&L_h_guai.flag==1)//&&(real_distance[imgInfo.top]-real_distance[R_h_guai.row])>25&&imgInfo.Both_lose==0 
{
 
            if(MAX(R_h_guai.row,L_h_guai.row)>real_distance_to_row(40)&&MAX(R_h_guai.row,L_h_guai.row)<real_distance_to_row(10)&&
            (real_distance[imgInfo.top]-real_distance[MAX(R_h_guai.row,L_h_guai.row)])>60&&real_distance[imgInfo.top]>100) 
            {
                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[MAX(R_h_guai.row,L_h_guai.row)]+12);//red_find_y到R_h_guai.row（*4)
               red_find_y1=MAX(R_h_guai.row,L_h_guai.row);
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right- red_find_x,red_find_y1 - red_find_y);
                // cv::Rect rect(red_find_x,red_find_y,Right_Sideline[MAX(R_h_guai.row,L_h_guai.row)],red_find_y1 - red_find_y);
                // cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);
        
                //cv::circle(resizedFrame, bestCenter, 1, cv::Scalar(0, 255, 0), -1);
            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[MAX(R_h_guai.row,L_h_guai.row)]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<20)//&&abs(center.x-R_h_guai.column)<20)
                // Flag.Redblock=1;
               }
            } 


                
}





      if(R_h_guai.flag==1&&R_h_guai.row>real_distance_to_row(40)&&R_h_guai.row<real_distance_to_row(10)&&
      (real_distance[imgInfo.top]-real_distance[R_h_guai.row])>60&&real_distance[imgInfo.top]>100)
{


                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//94//320
               red_find_y=real_distance_to_row(real_distance[R_h_guai.row]+12);//red_find_y到R_h_guai.row（*4)
               red_find_y1=R_h_guai.row;
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
                // cv::Rect rect(red_find_x,red_find_y,Right_Sideline[R_h_guai.row],red_find_y1 - red_find_y);
                // cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);
        
                //cv::circle(resizedFrame, bestCenter, 1, cv::Scalar(0, 255, 0), -1);
                             
            if(!red_objects.empty())
               {
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[R_h_guai.row]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<15)//&&abs(center.x-R_h_guai.column)<20
                // Flag.Redblock=1;
               }


}

      else if(L_h_guai.flag==1&&L_h_guai.row>real_distance_to_row(40)&&L_h_guai.row<real_distance_to_row(10)
      &&(real_distance[imgInfo.top]-real_distance[L_h_guai.row])>60&&real_distance[imgInfo.top]>100)
{



                    Flag.Redblock=0;red_x_mid=0;red_y_mid=0;
               red_find_x=red_left;//Right_Sideline[L_h_guai.row]
               red_find_y=real_distance_to_row(real_distance[L_h_guai.row]+12);//
                            red_find_y1=L_h_guai.row;
               DetectRedBlock(resizedFrame,red_find_x,red_find_y,red_right - red_find_x,red_find_y1 - red_find_y);
            //    cv::Rect rect(red_find_x,red_find_y,Right_Sideline[L_h_guai.row],red_find_y1 - red_find_y);
            //    cv::rectangle(resizedFrame, rect, cv::Scalar(0, 255, 0), 1);

                            
            if(!red_objects.empty())
               { 
                // Flag.Redblock=1;
                red_x_mid=resize_cx;
                red_y_mid=resize_cy;
                err_picture=fabs(real_distance[red_y_mid]-real_distance[L_h_guai.row]);
                x_err_red=abs((Right_Sideline[red_y_mid]+Left_Sideline[red_y_mid])/2-red_x_mid);
                // if(err_picture<15)//&&abs(center.x-L_h_guai.column)<20
                // Flag.Redblock=1;
               }







                


}




            if(x_err_red>5&&imgInfo.R_straight_flag==0&&imgInfo.R_straight_flag==1)
            {
               Flag.small_rock =1;
            }

            if(x_err_red>5&&imgInfo.R_straight_flag==1&&imgInfo.R_straight_flag==0)
            {
               Flag.small_rock =2;
            }



    }


    if(Flag.small_rock ==1||Flag.small_rock ==2)
    {
        if(distance>10)
        {
            distance=0;
            Flag.small_rock =0;
        }
    }
}



float ramp_line,ramp_err;
void ramp(void)
{

//    float k = xielv_sideline(30, (int)Dir_Err[30], 45, (int)Dir_Err[45], 'k');
//    float b = xielv_sideline(30, (int)Dir_Err[30], 45, (int)Dir_Err[45], 'b');
if(Flag.ramp==0)
{
     
    if(imgInfo.top<20)
  {
        ramp_err=0;
            //  ramp_err=50;
       for(int i = imgInfo.top+5;i<55 ;i++)
    {
            //   if(fabs(Dir_Err[i]-k*i-b)>2)
            // {
            //        if(i<50||(Right_Sideline_flag[i]==1&&Left_Sideline_flag[i]==1))
            //            ramp_line++;
            // }
          ramp_err+=Dir_Err[i];
    }
       if(dl1x_distance_raw>50&&dl1x_distance_raw<1500&&fabs(ramp_err)<30)//ramp_line==0&&&&B<0.2
    {
           Flag.ramp=2;

    }

  }
}
// if(Flag.ramp==1)
// {
//     if(distance>30)
//  {
//         Flag.ramp=2;
//  }
// }
if(Flag.ramp==2)
{
    if(distance>100)
 {
        Flag.ramp=3;
        distance=0;

 }
}
if(Flag.ramp==3)
{
    if(distance>150)
 {
        Flag.ramp=0;
        distance=0;

 }
}
}
 /***************************************************检测红色矩形块******************************************************/

 std::vector<RedObject> red_objects;
 
 int red_area = 0;
  #define MIN_RED_AREA 0//最小红色区域面积 用于滤除红色噪点
 #define MIN_AREA_FOR_BARRIER 250 //障碍物面积
int red_points_num;
int alpha = 0.1;
cv::Point center(-1,-1);
cv::Point last_center(-1,-1);
 void DetectRedBlock(const cv::Mat &search_space,int roi_x,int roi_y,int width,int height)
{
    red_objects.clear();//先清元素
    red_area = 0;
    red_points_num = 0;
    if(search_space.empty() || lq_frame.empty())
    {
        printf("DetectRedBlock input is empty\n");
        return;
    }
    if(roi_x < 0 || roi_y < 0 || roi_x >= search_space.cols || roi_y >= search_space.rows)
        return;
    if(width <= 0 || height <= 0 || roi_x + width > search_space.cols ||
       roi_y + height > search_space.rows)
        return;

    // 提供者建议的直接链：在 320x240 BGR 原图上按 RGB 通道优势找红色。
    // 调用者依然用 94x60 赛道坐标限定搜索区，这里只做一次几何映射，
    // 不再在缩小图上求质心后乘 3.4/4 回推原图。
    const double raw_scale_x = static_cast<double>(lq_frame.cols) / search_space.cols;
    const double raw_scale_y = static_cast<double>(lq_frame.rows) / search_space.rows;
    const int raw_x0 = std::max(0, static_cast<int>(std::floor(roi_x * raw_scale_x)));
    const int raw_y0 = std::max(0, static_cast<int>(std::floor(roi_y * raw_scale_y)));
    const int raw_x1 = std::min(lq_frame.cols,
                                static_cast<int>(std::ceil((roi_x + width) * raw_scale_x)));
    const int raw_y1 = std::min(lq_frame.rows,
                                static_cast<int>(std::ceil((roi_y + height) * raw_scale_y)));
    if(raw_x1 <= raw_x0 || raw_y1 <= raw_y0)
        return;

    const cv::Rect raw_search_rect(raw_x0, raw_y0, raw_x1 - raw_x0, raw_y1 - raw_y0);
    const cv::Mat raw_search = lq_frame(raw_search_rect);
    cv::Mat red_mask(raw_search.rows, raw_search.cols, CV_8UC1, cv::Scalar(0));
    for(int y = 0; y < raw_search.rows; ++y)
    {
        const cv::Vec3b *src_row = raw_search.ptr<cv::Vec3b>(y);
        uint8_t *mask_row = red_mask.ptr<uint8_t>(y);
        for(int x = 0; x < raw_search.cols; ++x)
        {
            const int b = src_row[x][0];
            const int g = src_row[x][1];
            const int r = src_row[x][2];
            if(r > Flash.debug_rgb_r_min &&
               (r - g) > Flash.debug_rgb_rg_diff &&
               (r - b) > Flash.debug_rgb_rb_diff)
            {
                mask_row[x] = 255;
            }
        }
    }

    cv::Rect roi_rect;
    cv::Mat labels, stats, centroids;
    red_points_num = cv::countNonZero(red_mask);

    int best_label = -1, best_area = 0;
    int second_label = -1, second_area = 0;
    const int min_raw_area = std::max(
        RED_MIN_AREA_KEEP,
        static_cast<int>(std::ceil(RED_MIN_AREA_KEEP * raw_scale_x * raw_scale_y)));
    if (red_points_num > 0)
    {
        const int label_count = cv::connectedComponentsWithStats(
            red_mask, labels, stats, centroids, 8, CV_32S, cv::CCL_GRANA);
        for (int label = 1; label < label_count; ++label)
        {
            const int area = stats.at<int>(label, cv::CC_STAT_AREA);
            const int raw_cy = cvRound(centroids.at<double>(label, 1)) + raw_search_rect.y;
            const int search_cy = cvRound(raw_cy / raw_scale_y);
            if (area < min_raw_area || search_cy < 15)
                continue;
            if (area > best_area)
            {
                second_label = best_label;
                second_area = best_area;
                best_label = label;
                best_area = area;
            }
            else if (area > second_area)
            {
                second_label = label;
                second_area = area;
            }
        }
    }

    const int final_label =
        (best_label >= 0 && second_label >= 0)
            ? ((centroids.at<double>(best_label, 1) >= centroids.at<double>(second_label, 1)) ? best_label : second_label)
            : best_label;
    if (final_label >= 0)
    {
        center = cv::Point(
            cvRound(centroids.at<double>(final_label, 0)) + raw_search_rect.x,
            cvRound(centroids.at<double>(final_label, 1)) + raw_search_rect.y);
        resize_cx = std::clamp(cvRound(center.x / raw_scale_x), 0, search_space.cols - 1);
        resize_cy = std::clamp(cvRound(center.y / raw_scale_y), 0, search_space.rows - 1);

        // Board/Brick 路由只要求直接物理 ROI64 合法；ROI80 由 Board 分支单独检查。
        if (center.x < DENSITY_CENTER_X_MIN || center.x > DENSITY_CENTER_X_MAX ||
            center.y < DENSITY_CENTER_Y_MIN || center.y > DENSITY_CENTER_Y_MAX)
        {
            static int s_edge_skip = 0;
            if ((s_edge_skip++ % 30) == 0)
            {
                printf("[REC] density64 edge skip center=(%d,%d) area=%d "
                       "valid_x[%d,%d] y[%d,%d]\n",
                       center.x, center.y, stats.at<int>(final_label, cv::CC_STAT_AREA),
                       DENSITY_CENTER_X_MIN, DENSITY_CENTER_X_MAX,
                       DENSITY_CENTER_Y_MIN, DENSITY_CENTER_Y_MAX);
            }
            center = cv::Point(-1, -1);
            last_center = cv::Point(-1, -1);
            return;
        }

        if(last_center != cv::Point(-1, -1))
        {
            int dx = center.x - last_center.x;
            int dy = center.y - last_center.y;
            double dist = sqrt(dx * dx + dy * dy);
            if(dist<16)
            {
                center = last_center;
            }
        }
        last_center = center;
        red_area = stats.at<int>(final_label, cv::CC_STAT_AREA);
        red_objects.push_back({center,red_area});
    }
    else{
        center = cv::Point(-1,-1);
        last_center = cv::Point(-1,-1);
        resize_cx = 0;
        resize_cy = 0;
    }

    if(!red_objects.empty()&&Flag.infer ==1)
    {   
   auto &obj = red_objects[0];

        if (obj.center.x < 0 || obj.center.y < 0 || 
        obj.center.x >= lq_frame.cols || obj.center.y >= lq_frame.rows||obj.center.y < 0||obj.center.x < 0) 
        {
        return;
        }

        const cv::Rect density_rect(
            obj.center.x - DENSITY_MARKER_X + ROI_SHIFT_X,
            obj.center.y - DENSITY_MARKER_Y + ROI_SHIFT_Y,
            DENSITY_ROI_SIZE, DENSITY_ROI_SIZE);
        if (density_rect.x < 0 || density_rect.y < 0 ||
            density_rect.x + density_rect.width > lq_frame.cols ||
            density_rect.y + density_rect.height > lq_frame.rows)
        {
            printf("[RED_SPLIT] direct density64 invalid center=(%d,%d) rect=%d,%d,%d,%d\n",
                   obj.center.x, obj.center.y,
                   density_rect.x, density_rect.y,
                   density_rect.width, density_rect.height);
            return;
        }
        cv::Mat density_roi64 = lq_frame(density_rect);
        if (density_roi64.empty() ||
            density_roi64.rows != DENSITY_ROI_SIZE ||
            density_roi64.cols != DENSITY_ROI_SIZE ||
            density_roi64.type() != CV_8UC3)
        {
            printf("[RED_SPLIT] direct density64 preparation failed roi=%dx%d type=%d\n",
                   density_roi64.cols, density_roi64.rows, density_roi64.type());
            return;
        }
        if (g_run_mode == RUN_MODE_CAMERA_FOCUS)
        {
            s_focus_red_center = obj.center;
            s_focus_direct64_valid = true;
            s_focus_direct64_rect = density_rect;
            density_roi64.copyTo(s_focus_direct64_input);
        }

        float white_ratio = 0.0f;
        const bool white_ratio_valid =
            red_brick_filter::CalcWhiteRatio(density_roi64, white_ratio);
        if (!white_ratio_valid)
        {
            printf("[RED_SPLIT_DIAG] white ratio unavailable density_roi=%dx%d type=%d; "
                   "inference unchanged\n",
                   density_roi64.cols, density_roi64.rows, density_roi64.type());
        }
        if (IsRedRouteEnabledMode())
        {
            if (s_red_route_observed_this_frame)
            {
                red_objects.clear();
                return;
            }
            s_red_route_observed_this_frame = true;
            const int lane_row = std::clamp(resize_cy, 0, LCDH_1 - 1);
            const int lane_center =
                (Left_Sideline[lane_row] + Right_Sideline[lane_row]) / 2;
            const int signed_side_error = resize_cx - lane_center;
            red_event::RedObservation observation;
            observation.valid = white_ratio_valid;
            observation.white_ratio = white_ratio;
            observation.signed_side_error = static_cast<std::int16_t>(
                std::clamp(signed_side_error, -32768, 32767));
            red_event::RedRouteResult route =
                s_red_event_router.Observe(observation);
            PublishRedRouteResult(route, white_ratio_valid, white_ratio,
                                  signed_side_error);
            if (route.kind == red_event::RedEventKind::kBoardCandidate &&
                !route.run_board_ncnn)
            {
                route = s_red_event_router.LockBoard(
                    red_event::BoardRoute::kStraight);
                PublishRedRouteResult(route, white_ratio_valid, white_ratio,
                                      signed_side_error);
                Flag.picture = 5;
                Flag.Redblock = 0;
                Flag.infer = 0;
                Flag.supply = 0;
                Flag.vehicle = 0;
                Flag.weapon = 0;
                distance_picture = 0;
                red_objects.clear();
                return;
            }
            if (route.new_event && IsBrickLocked(route.kind))
            {
                if (IsRedRouteDiagnosticMode() &&
                    s_red_brick_detour.Start(route.avoid_direction, MonotonicMs()))
                {
                    s_red_detour_command = s_red_brick_detour.command();
                    printf("[BRICK_DETOUR_DIAG] start event=%u brick=%s avoid=%s "
                           "state=%s control_consumed=0\n",
                           route.event_id, red_event::ToString(route.brick_side),
                           red_event::ToString(route.avoid_direction),
                           red_event::ToString(s_red_brick_detour.state()));
                }
                else if (g_run_mode == RUN_MODE_TRACK)
                {
                    // Brick leaves the existing target-board pre-stage before control starts.
                    Flag.picture = 0;
                    Flag.Redblock = 0;
                    Flag.infer = 0;
                    Flag.supply = 0;
                    Flag.vehicle = 0;
                    Flag.weapon = 0;
                    distance_picture = 0;
                    red_event::BrickDetourRequest request;
                    request.event_id = route.event_id;
                    request.direction = route.avoid_direction;
                    const bool published =
                        red_event::PublishBrickDetourRequest(request);
                    printf("[BRICK_DETOUR_TRACK] request event=%u brick=%s avoid=%s "
                           "published=%u\n",
                           route.event_id, red_event::ToString(route.brick_side),
                           red_event::ToString(route.avoid_direction),
                           published ? 1u : 0u);
                }
            }
            if (!route.run_board_ncnn)
            {
                if (g_run_mode == RUN_MODE_CAMERA_FOCUS)
                {
                    s_focus_distance_cm =
                        real_distance[MAX(L_h_guai.row, R_h_guai.row)];
                }
                red_objects.clear();
                return;
            }
        }

        // Board branch only: provider model keeps physical ROI80 -> MODEL64.
        if(!GenerateROI(obj.center, roi_rect, lq_frame)){
            static int s_roi_fail = 0;
            if ((s_roi_fail++ % 15) == 0)
            {
                printf("[REC] board ROI80 fail center=(%d,%d) img=%dx%d shift=%d,%d "
                       "need x[%d,%d] y[%d,%d]\n",
                       obj.center.x, obj.center.y, lq_frame.cols, lq_frame.rows,
                       ROI_SHIFT_X, ROI_SHIFT_Y,
                       RED_CENTER_X_MIN, RED_CENTER_X_MAX,
                       RED_CENTER_Y_MIN, RED_CENTER_Y_MAX);
                // 诊断：越界时也存全图，便于看红点位置
                if (g_run_mode == RUN_MODE_IMAGE_TEST && s_image_test_capture_count < kImageTestCaptureLimit)
                {
                    char capture_dir[64] = {};
                    snprintf(capture_dir, sizeof(capture_dir), "/tmp/4m_lq_debug/%d", (int)getpid());
                    mkdir("/tmp/4m_lq_debug", 0700);
                    mkdir(capture_dir, 0700);
                    char path[96] = {};
                    snprintf(path, sizeof(path), "%s/fail_frame_%02d_c%d_%d.png",
                             capture_dir, s_image_test_capture_count,
                             obj.center.x, obj.center.y);
                    cv::Mat ann = lq_frame.clone();
                    cv::circle(ann, obj.center, 4, cv::Scalar(0, 255, 255), -1);
                    if (cv::imwrite(path, ann))
                    {
                        printf("[REC] fail capture: %s\n", path);
                        s_image_test_capture_count++;
                    }
                }
            }
            return;
        }
        if (roi_rect.x < 0 || roi_rect.y < 0 ||
            roi_rect.width <= 0 || roi_rect.height <= 0 ||
            roi_rect.x + roi_rect.width > lq_frame.cols ||
            roi_rect.y + roi_rect.height > lq_frame.rows)
        {
                std::cout << "BAD ROI: "
              << roi_rect << " | img: "
              << lq_frame.cols << "x"
              << lq_frame.rows << std::endl;
                return;
        }
            if (g_run_mode == RUN_MODE_CAMERA_FOCUS)
            {
                s_focus_roi_valid = true;
                s_focus_roi_rect = roi_rect;
            }
            cv::Mat candidate_roi80 = lq_frame(roi_rect);
              if(candidate_roi80.empty() ||
                 candidate_roi80.rows != CLASSIFIER_SOURCE_ROI_SIZE ||
                 candidate_roi80.cols != CLASSIFIER_SOURCE_ROI_SIZE ||
                 candidate_roi80.type() != CV_8UC3)
                 {
                    return;
                 }
                 cv::Mat model_input64;
                 cv::resize(candidate_roi80, model_input64,
                            cv::Size(LQ_NCNN::kInputWidth, LQ_NCNN::kInputHeight),
                            0.0, 0.0, cv::INTER_LINEAR);
                 if (model_input64.empty() ||
                     model_input64.rows != LQ_NCNN::kInputHeight ||
                     model_input64.cols != LQ_NCNN::kInputWidth ||
                     model_input64.type() != CV_8UC3)
                 {
                     printf("[REC] model input preparation failed roi80=%dx%d type=%d\n",
                            candidate_roi80.cols, candidate_roi80.rows, candidate_roi80.type());
                     return;
                 }
                 if (g_run_mode == RUN_MODE_CAMERA_FOCUS)
                 {
                     model_input64.copyTo(s_focus_model_input);
                 }
                 real_picture_distance = real_distance[MAX(L_h_guai.row, R_h_guai.row)];
                 printf("[REC] ROI ready dist=%.1f redn=%zu infer=%u physical=%dx%d "
                        "density=%dx%d model=%dx%d shift=%d,%d white_ratio=%.4f white_valid=%u "
                        "white_threshold=%.2f route=gte-board/lt-brick enabled_mode=%d\n",
                        real_picture_distance, red_objects.size(), (unsigned)Flag.infer,
                        candidate_roi80.cols, candidate_roi80.rows,
                        density_roi64.cols, density_roi64.rows,
                        model_input64.cols, model_input64.rows,
                        ROI_SHIFT_X, ROI_SHIFT_Y,
                        white_ratio_valid ? white_ratio : -1.0f,
                        white_ratio_valid ? 1u : 0u,
                        s_red_event_router.config().white_ratio,
                        static_cast<int>(g_run_mode));

                 // 大佬 NCNN：supplies/vehicle/weapon，保持当前车历史 Flag 映射。
                 int class_idx = -1;
                 float confidence = 0.f;
                 long infer_us = 0;
                 float class_probs[3] = {};
                 if (!classifier.Infer(model_input64, class_idx, confidence, infer_us, class_probs))
                 {
                     printf("[REC] NCNN inference failed: %s\n", classifier.LastError());
                     return;
                 }
                 const char *result = classifier.Label(class_idx);
                 printf("[REC] 结果:%s conf=%.3f us=%ld (supplies/vehicle/weapon)\n",
                        result, confidence, infer_us);
                 if (g_run_mode == RUN_MODE_CAMERA_FOCUS)
                 {
                     s_focus_result_valid = true;
                     s_focus_class_idx = class_idx;
                     s_focus_confidence = confidence;
                     s_focus_distance_cm = real_picture_distance;

                     if (s_focus_direct64_valid)
                     {
                         int direct_class_idx = -1;
                         float direct_confidence = 0.0f;
                         long direct_infer_us = 0;
                         float direct_probs[3] = {};
                         if (classifier.Infer(s_focus_direct64_input,
                                              direct_class_idx,
                                              direct_confidence,
                                              direct_infer_us,
                                              direct_probs))
                         {
                             s_focus_direct64_result_valid = true;
                             s_focus_direct64_class_idx = direct_class_idx;
                             s_focus_direct64_confidence = direct_confidence;
                             printf("[FOCUS_D64] result=%s conf=%.3f probs=%.6f,%.6f,%.6f "
                                    "us=%ld rect=%d,%d,64,64 marker=32,50\n",
                                    classifier.Label(direct_class_idx),
                                    direct_confidence,
                                    direct_probs[0], direct_probs[1], direct_probs[2],
                                    direct_infer_us,
                                    s_focus_direct64_rect.x,
                                    s_focus_direct64_rect.y);
                         }
                     }
                 }
                 SaveImageTestCapture(lq_frame, candidate_roi80, density_roi64,
                                      model_input64,
                                      obj.center, roi_rect,
                                      roi_x, roi_y, width, height,
                                      resize_cx, resize_cy,
                                      white_ratio_valid, white_ratio,
                                      class_idx, confidence, infer_us, class_probs);

                 if (dataset_capture::IsActive())
                 {
                     dataset_capture::FrameSample capture_sample;
                     capture_sample.frame_bgr = lq_frame;
                     capture_sample.roi80_bgr = candidate_roi80;
                     capture_sample.model_input64_bgr = model_input64;
                     capture_sample.red_center = obj.center;
                     capture_sample.roi_rect = roi_rect;
                     capture_sample.frame_id = g_image_frame_id + 1;
                     capture_sample.distance_cm = real_picture_distance;
                     capture_sample.white_ratio_valid = white_ratio_valid;
                     capture_sample.white_ratio = white_ratio;
                     capture_sample.class_idx = class_idx;
                     capture_sample.confidence = confidence;
                     capture_sample.infer_us = infer_us;
                     for (int probability_index = 0; probability_index < 3;
                          ++probability_index)
                     {
                         capture_sample.class_probs[probability_index] =
                             class_probs[probability_index];
                     }
                     capture_sample.exposure_config = Flash.camera_exposure;
                     capture_sample.exposure_mode = uvc_dev.get_auto_exposure_mode();
                     capture_sample.exposure_value = uvc_dev.get_current_exposure();
                     dataset_capture::OnValidFrame(capture_sample);
                 }

                 if (confidence < g_picture_conf_min)
                     return;

                 if (IsRedRouteEnabledMode())
                 {
                     red_event::BoardRoute board_route = red_event::BoardRoute::kNone;
                     if (class_idx == 0)
                         board_route = red_event::BoardRoute::kRight;
                     else if (class_idx == 1)
                         board_route = red_event::BoardRoute::kStraight;
                     else if (class_idx == 2)
                         board_route = red_event::BoardRoute::kLeft;
                     PublishRedRouteResult(
                         s_red_event_router.LockBoard(board_route),
                         white_ratio_valid, white_ratio, s_red_signed_side_error);
                 }

                 if (class_idx == 0) // supplies
                 {
                     Flag.supply++;
                     Flag.weapon = 0;
                     Flag.vehicle = 0;
                 }
                 else if (class_idx == 2) // weapon
                 {
                     Flag.supply = 0;
                     Flag.weapon++;
                     Flag.vehicle = 0;
                 }
                 else if (class_idx == 1) // vehicle
                 {
                     Flag.supply = 0;
                     Flag.weapon = 0;
                     Flag.vehicle++;
                 }
    }

}


// 大佬 80->64 推导裁图：红心 (cx,cy) -> (cx-40,cy-63,80,80)，包含红色区域。
bool GenerateROI(const cv::Point &center, cv::Rect &roi, const cv::Mat &src)
{
    if (center.x < 0 || center.y < 0) {
        return false;
    }
    
    if (src.empty()) {
        return false;
    }
    roi_x_1 = center.x - CLASSIFIER_SOURCE_MARKER_X + ROI_SHIFT_X;
    roi_y_1 = center.y - CLASSIFIER_SOURCE_MARKER_Y + ROI_SHIFT_Y;

    if (roi_x_1 < 0 || roi_y_1 < 0)
        return false;

    if (roi_x_1 + ROI_SIZE > src.cols)
        return false;

    if (roi_y_1+ ROI_SIZE > src.rows)
        return false;

    roi = cv::Rect(roi_x_1, roi_y_1, ROI_SIZE, ROI_SIZE);
    return true;
}
/***************************************************红色标注******************************************************/
enum ClassType{
    CLASS_SUPPLIES = 0,
    CLASS_VEHICLE,
    CLASS_WEAPON,
    CLASS_UNKNOWN
};

ClassType GetClassID(const std::string &cls){
    if(cls == "supplies")
        return CLASS_SUPPLIES;
    else if(cls == "vehicle")
        return CLASS_VEHICLE;
    else if(cls == "weapon")
        return CLASS_WEAPON;
    return CLASS_UNKNOWN;
}
ClassType id;

#define SEARCH_BOTTOM_RATIO          1.0f   // 1.0=全图搜索；0.5=只搜下半部分
#define MIN_RED_AREA                 40     // 最小红块面积

// HSV 红色阈值
#define LOWER_RED_H1                 0
#define UPPER_RED_H1                 10
#define LOWER_RED_H2                 160
#define UPPER_RED_H2                 179
#define LOWER_RED_S                  120
#define LOWER_RED_V                  70

#define ROI_OFFSET_X                 0
#define ROI_OFFSET_Y                 -10
cv::Rect crop_rect;
cv::Mat src_img;
cv::Mat roi_img;         // 纯ROI
cv::Rect red_rect;       // 红块框
cv::Point roi_center;    // ROI中心
//限幅函数
static inline int clamp_int(int v, int low, int high)
{
    if (v < low)  return low;
    if (v > high) return high;
    return v;
}

 bool detect_red_and_crop_roi(const cv::Mat& src_img,
                             cv::Mat& roi_img,
                             cv::Rect& best_rect,
                             cv::Rect& crop_rect,
                             cv::Point& roi_center)
{
    roi_img.release();
    best_rect = cv::Rect();
    crop_rect = cv::Rect();
    roi_center = cv::Point(-1, -1);

    if (src_img.empty()) return false;

    // 只在指定区域搜索
    int roi_y = static_cast<int>(src_img.rows * (1.0f - SEARCH_BOTTOM_RATIO));
    roi_y = clamp_int(roi_y, 0, src_img.rows - 1);

    cv::Rect search_roi(0, roi_y, src_img.cols, src_img.rows - roi_y);
    cv::Mat src_search = src_img(search_roi);

    // 转HSV找红色
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(src_search, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv,
            Scalar(LOWER_RED_H1, LOWER_RED_S, LOWER_RED_V),
            Scalar(UPPER_RED_H1, 255, 255),
            mask1);

    cv::inRange(hsv,
            Scalar(LOWER_RED_H2, LOWER_RED_S, LOWER_RED_V),
            Scalar(UPPER_RED_H2, 255, 255),
            mask2);

    mask = mask1 | mask2;

    // 开闭运算去噪         打开后会降低图像帧率 20帧左右
    // static cv::Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    // cv::morphologyEx(mask, mask, MORPH_OPEN, kernel);
    // cv::morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    // 查找轮廓
    std::vector<std::vector<Point>> contours;
    cv::findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return false;

    // 选择面积最大的红块
    double max_area = 0.0;
    cv::Rect max_rect;

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area < MIN_RED_AREA) continue;

        cv::Rect rect = cv::boundingRect(contours[i]);
        if (area > max_area)
        {
            max_area = area;
            max_rect = rect;
        }
    }

    if (max_area < MIN_RED_AREA) return false;

    // 转回原图坐标
    max_rect.y += roi_y;
    best_rect = max_rect;

    int red_cx = best_rect.x + best_rect.width / 2;// 红块中心x坐标
    int red_cy = best_rect.y + best_rect.height / 2;// 红块中心y坐标

    int cx = red_cx + ROI_OFFSET_X;
    int cy = red_cy + ROI_OFFSET_Y;
    roi_center = Point(cx, cy);// ROI中心点
    //printf("cx:%d,cy:%d\n",cx,cy);

    roix1 = cx - REC_ROI_WIDTH / 2;
    roiy1 = cy - REC_ROI_HEIGHT / 2;

    roix1 = clamp_int(roix1, 0, src_img.cols - REC_ROI_WIDTH);//ROI框左上和角x坐标
    roiy1 = clamp_int(roiy1, 0, src_img.rows - REC_ROI_HEIGHT);//ROI框左上角y坐标

    crop_rect = cv::Rect(roix1, roiy1, REC_ROI_WIDTH, REC_ROI_HEIGHT);

    if (crop_rect.x < 0 || crop_rect.y < 0 ||
        crop_rect.x + crop_rect.width > src_img.cols ||
        crop_rect.y + crop_rect.height > src_img.rows)
    {
        return false;
    }

    roi_img = src_img(crop_rect);

    return true;
}
/***************************************************图像处理******************************************************/
void ImageDeal()
{
    // 相机失败时限流日志并直接返回，禁止沿用旧帧。
    if (!g_camera_ok)
    {
        static uint32_t s_cam_fail_n = 0;
        if ((++s_cam_fail_n % 3000u) == 1u)
            printf("[ImageDeal] camera not ok, skip frame refresh (log throttled)\n");
        return;
    }
    if (uvc_dev.wait_image_refresh() != 0)
    {
        return;
    }
    const cv::Mat source_frame = uvc_dev.get_frame_bgr();
    if(source_frame.empty()){
        return; 
    }
    // 复制逐飞驱动的原生 320x240 BGR 帧，保持红心、ROI 与训练数据同一像素合同。
    source_frame.copyTo(lq_frame);

    if (g_run_mode == RUN_MODE_CAMERA_FOCUS)
    {
        s_focus_result_valid = false;
        s_focus_class_idx = -1;
        s_focus_confidence = 0.0f;
        s_focus_distance_cm = 0.0f;
        s_focus_roi_valid = false;
        s_focus_roi_rect = cv::Rect();
        s_focus_red_center = cv::Point();
        s_focus_model_input.release();
        s_focus_direct64_valid = false;
        s_focus_direct64_rect = cv::Rect();
        s_focus_direct64_input.release();
        s_focus_direct64_result_valid = false;
        s_focus_direct64_class_idx = -1;
        s_focus_direct64_confidence = 0.0f;
    }
    if (IsRedRouteEnabledMode())
    {
        s_red_signed_side_error = 0;
        s_red_route_observed_this_frame = false;
    }

      cv::resize(lq_frame,resizedFrame,cv::Size(LCDW_1,LCDH_1),0,0,cv::INTER_AREA);

    // RedBlockProcess(resizedFrame);
    // camera_server.update_frame_mat(lq_frame);//打开图传服务器

      cv::cvtColor(resizedFrame,grayFrame,cv::COLOR_BGR2GRAY);

    //   cv::warpAffine(grayFrame, translatedFrame, translationMatrix, grayFrame.size(), 
    //                  cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

//   //  将OpenCV图像数据复制到图像数组 采用memcpy函数加快处理速度
    for (int i = 0; i < LCDH_1; i++)
    {
        uint8_t *p = grayFrame.ptr<uint8_t>(i);
        for(int w = 0; w < LCDW_1; w++)
        {
            Image_Use[i][w] = p[w];

        }
    }
         Get01change_dajin();

        // my_sobel(Image_Zip,Image_Use); //压缩后的图像数组,

        // my_sobel_dajin(Image_Zip,Image_Use);

        imgInfoInit();

        Get_ImageTop();

        Draw_BlackSideline(Image_Use);//画边线

        Find_Sideline(imgInfo.bottom-1,imgInfo.top+ 1);//找边线

        // 只冻结日志入口状态；视觉判断继续直接读 Flag，与 ae3379a 一致。
        g_huandao_L_geom = Flag.Huandao_L;
        g_huandao_R_geom = Flag.Huandao_R;

        if(Flag.Huandao_L>0||Flag.Huandao_R>0)
        Find_Guaidian();  //找拐点
        else
        Find_Guaidian1();  //找拐点

        straight_judge();

        zebra_corssing();

        if(Flag.Zebra_cross==3)
        {
        ramp();
        }

        // A locked brick event exclusively owns the red target until detour DONE.
        // Prevent the legacy board pre-stage from restoring picture=2 mid-detour.
        const bool brick_route_owns_red_target =
            g_run_mode == RUN_MODE_TRACK && IsBrickLocked(s_red_route_result.kind);
        if (brick_route_owns_red_target)
        {
            Flag.picture = 0;
            Flag.Redblock = 0;
            Flag.infer = 0;
            Flag.supply = 0;
            Flag.vehicle = 0;
            Flag.weapon = 0;
            distance_picture = 0;
            red_objects.clear();
        }
        else
        {
            picture();
        }

        // mode=30 锁电机静态验识别：0 速常无拐点，作者 picture() 不会扫红块。
        // 仅在锁电机诊断模式下全图扫红块并开 infer；TRACK(10) 仍严格走作者拐点条件。
        if (IsRedTargetProcessingEnabled() &&
            (g_run_mode == RUN_MODE_IMAGE_TEST || g_run_mode == RUN_MODE_CAMERA_FOCUS) &&
            Flag.Huandao_L == 0 && Flag.Huandao_R == 0)
        {
            Flag.infer = 1;
            // 全图红扫描（0速无拐点时 picture() 不会扫）
            if (resizedFrame.channels() >= 3)
            {
                DetectRedBlock(resizedFrame, 2, 2, LCDW_1 - 4, LCDH_1 - 4);
            }
            else
            {
                static int s_ch_warn = 0;
                if ((s_ch_warn++ % 60) == 0)
                    printf("[REC] resizedFrame channels=%d (need 3 for red)\n",
                           resizedFrame.channels());
            }
            const bool board_branch =
                s_red_route_result.kind == red_event::RedEventKind::kBoardCandidate ||
                s_red_route_result.kind == red_event::RedEventKind::kBoardLocked;
            if (!red_objects.empty() && board_branch)
            {
                Flag.Redblock = 1;
                Flag.picture = 2;
            }
            UpdateRedDetourDiagnostic();
            // 低频诊断：红阈值下像素数 + 周期性全图存盘（无目标时也能看构图）
            {
                static int s_rd = 0;
                if ((s_rd++ % 30) == 0)
                {
                    int rmax = 0, rgmax = 0;
                    if (lq_frame.channels() >= 3)
                    {
                        for (int y = 0; y < lq_frame.rows; y += 2)
                        {
                            const cv::Vec3b *p = lq_frame.ptr<cv::Vec3b>(y);
                            for (int x = 0; x < lq_frame.cols; x += 2)
                            {
                                int g = p[x][1], r = p[x][2];
                                if (r > rmax)
                                    rmax = r;
                                if (r - g > rgmax)
                                    rgmax = r - g;
                            }
                        }
                    }
                    printf("[REC] red_scan_fullres thr r>%d rg/rb>%d/%d search_hit=%d "
                           "rmax=%d rgmax=%d "
                           "redn=%zu Lh=%u Rh=%u\n",
                           Flash.debug_rgb_r_min, Flash.debug_rgb_rg_diff,
                           Flash.debug_rgb_rb_diff, red_points_num, rmax, rgmax,
                           red_objects.size(),
                           (unsigned)L_h_guai.flag, (unsigned)R_h_guai.flag);

                    // 每 ~30 帧存一张 scene 全图，最多 5 张
                    static int s_scene_n = 0;
                    if (s_scene_n < 5 && !lq_frame.empty())
                    {
                        char capture_dir[64] = {};
                        snprintf(capture_dir, sizeof(capture_dir),
                                 "/tmp/4m_lq_debug/%d", (int)getpid());
                        mkdir("/tmp/4m_lq_debug", 0700);
                        mkdir(capture_dir, 0700);
                        char path[96] = {};
                        snprintf(path, sizeof(path), "%s/scene_%02d_exp%d.png",
                                 capture_dir, s_scene_n, Flash.camera_exposure);
                        if (cv::imwrite(path, lq_frame))
                        {
                            printf("[REC] scene capture: %s\n", path);
                            s_scene_n++;
                        }
                    }
                }
            }
        }

        if (IsRedRouteEnabledMode() && !s_red_route_observed_this_frame)
        {
            PublishRedRouteResult(s_red_event_router.ObserveMissing(), false, 0.0f, 0);
            s_red_route_observed_this_frame = true;
        }
        if (IsRedRouteEnabledMode())
            MaybeCompleteRedEvent();

        if(Flag.Huandao_L==0&&Flag.Huandao_R==0)
        small_rock();
        // }

        // if(Flag.Huandao_L!=1&&Flag.Huandao_R!=1)


       Huandao_R_imu();
       Huandao_L_imu();


       if(Flag.Huandao_L!=1&&Flag.Huandao_R!=1&&Flag.Huandao_L!=2&&Flag.Huandao_R!=2&&Flag.Huandao_L!=3&&Flag.Huandao_R!=3
       &&Flag.Huandao_L!=4&&Flag.Huandao_R!=4&&Flag.Huandao_L!=5&&Flag.Huandao_R!=5&&Flag.Huandao_L!=6&&Flag.Huandao_R!=6)
        Buxian();

        Find_Midline();

     //   dynamic_forward();



        Err_Sum();

        if (g_run_mode == RUN_MODE_CAMERA_FOCUS)
        {
            UpdateCameraFocusStream(lq_frame);
        }
        // HTTP :8080 图传：二值图+边线+阈值文字（调 threshold_static 用）
        else
        {
            cv::Mat bin(LCDH_1, LCDW_1, CV_8UC1);
            for (int y = 0; y < LCDH_1; ++y)
                std::memcpy(bin.ptr<uint8_t>(y), Image_Use[y], (size_t)LCDW_1);
            cv::Mat vis;
            cv::cvtColor(bin, vis, cv::COLOR_GRAY2BGR);
            for (int y = 0; y < LCDH_1; ++y)
            {
                const int lx = Left_Sideline[y];
                const int mx = Mid_Line[y];
                const int rx = Right_Sideline[y];
                if (lx >= 0 && lx < LCDW_1)
                    vis.at<cv::Vec3b>(y, lx) = cv::Vec3b(0, 255, 0);
                if (mx >= 0 && mx < LCDW_1)
                    vis.at<cv::Vec3b>(y, mx) = cv::Vec3b(0, 255, 255);
                if (rx >= 0 && rx < LCDW_1)
                    vis.at<cv::Vec3b>(y, rx) = cv::Vec3b(0, 0, 255);
            }
            char thr_txt[56];
            std::snprintf(thr_txt, sizeof(thr_txt), "thr=%u min=%u D=%.1f",
                          (unsigned)Threshold, (unsigned)Threshold_static, Dir_err);
            cv::putText(vis, thr_txt, cv::Point(2, 12),
                        cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(255, 180, 0), 1);
            camera_server.update_frame_mat(vis);
        }

        // 本帧视觉快照已完整更新，供 PIT 新鲜度检查
        image_mark_success_frame();

        protect();
}
