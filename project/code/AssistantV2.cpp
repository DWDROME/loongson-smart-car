// 迁自 Car_Round/code/AssistantV2.cpp
// 图像：grayFrame 94x60；边线：Left/Mid/Right_Sideline 按行直发。
// 协议库：libraries/zf_components/seekfree_assistant/seekfree_assistant_v2.*（V2，非 V1）。
#include "AssistantV2.hpp"

#include "zf_common_headfile.hpp"
#include "seekfree_assistant_v2.hpp"  // 必须 V2
#include "zf_driver_tcp_client.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{
constexpr const char *k_default_ip = SERVER_IP;
constexpr uint32 k_default_port = PORT;
constexpr int k_default_frame_div = 5;
constexpr int k_default_send_retry = 2;
constexpr int k_default_retry_interval_ms = 1;

// 当前工程控制图像尺度（任务合同）
constexpr int k_img_w = LCDW_1; // 94
constexpr int k_img_h = LCDH_1; // 60

constexpr uint16 k_color_left = 0x07E0;
constexpr uint16 k_color_mid = 0xF800;
constexpr uint16 k_color_right = 0x001F;

struct assistant_v2_state_t
{
    bool connected;
    bool send_image;
    uint32 frame_div;
    const char *ip;
    uint32 port;
    uint8 send_retry;
    uint16 retry_interval_ms;
    zf_driver_tcp_client tcp;
    seekfree_assistant_v2_camera_struct camera;
    seekfree_assistant_v2_camera_boundary_struct left_boundary;
    seekfree_assistant_v2_camera_boundary_struct mid_boundary;
    seekfree_assistant_v2_camera_boundary_struct right_boundary;
    uint8 image_buffer[k_img_h][k_img_w];
    seekfree_assistant_v2_camera_boundary_uint8_struct left_pts[k_img_h];
    seekfree_assistant_v2_camera_boundary_uint8_struct mid_pts[k_img_h];
    seekfree_assistant_v2_camera_boundary_uint8_struct right_pts[k_img_h];
};

assistant_v2_state_t g_assistant;

static int clamp_i(int value, int low, int high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

static int read_env_int_clamped(const char *name, int default_value, int low, int high)
{
    const char *text = std::getenv(name);
    if (text == nullptr || text[0] == '\0')
        return default_value;

    errno = 0;
    char *end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (errno != 0 || end == text || (end != nullptr && end[0] != '\0'))
    {
        std::printf("assistant v2 invalid %s=%s, use %d\r\n", name, text, default_value);
        return default_value;
    }
    if (value < low)
        return low;
    if (value > high)
        return high;
    return static_cast<int>(value);
}

static bool read_env_flag(const char *name, bool default_value)
{
    const char *text = std::getenv(name);
    if (text == nullptr || text[0] == '\0')
        return default_value;
    return std::strcmp(text, "1") == 0 ||
           std::strcmp(text, "true") == 0 ||
           std::strcmp(text, "TRUE") == 0 ||
           std::strcmp(text, "on") == 0 ||
           std::strcmp(text, "ON") == 0;
}

// 边线按行索引直接发满 k_img_h 点（与控制数组一致）。
// 曾做过 top 范围/flag 过滤的显示试验，已确认根因在二值阈值，过滤不进主线。
static void copy_sideline_to_points(const int *line,
                                    seekfree_assistant_v2_camera_boundary_uint8_struct pts[k_img_h])
{
    for (int y = 0; y < k_img_h; ++y)
    {
        pts[y].x = static_cast<uint8>(clamp_i(line[y], 0, k_img_w - 1));
        pts[y].y = static_cast<uint8>(y);
    }
}

static void update_boundaries(void)
{
    copy_sideline_to_points(Left_Sideline, g_assistant.left_pts);
    copy_sideline_to_points(Mid_Line, g_assistant.mid_pts);
    copy_sideline_to_points(Right_Sideline, g_assistant.right_pts);
}

static void copy_gray_image(void)
{
    // 原因：ImageDeal 把灰度写入 Image_Use 后立刻大津二值化；Image_Zip 当前链路根本不更新。
    // 上位机应发 grayFrame（resize 后 94x60 灰度），边线仍按二值图搜线结果叠加。
    if (!grayFrame.empty() && grayFrame.rows >= k_img_h && grayFrame.cols >= k_img_w &&
        grayFrame.type() == CV_8UC1)
    {
        for (int y = 0; y < k_img_h; ++y)
        {
            const uint8_t *row = grayFrame.ptr<uint8_t>(y);
            std::memcpy(g_assistant.image_buffer[y], row, static_cast<size_t>(k_img_w));
        }
        return;
    }
    // 兜底：至少发当前 Image_Use（多半已是二值），避免全黑
    std::memcpy(g_assistant.image_buffer[0], &Image_Use[0][0], static_cast<size_t>(k_img_w * k_img_h));
}

static uint32 assistant_v2_transfer(const uint8 *buff, uint32 length)
{
    if (!g_assistant.connected || buff == nullptr || length == 0)
        return 0;

    const uint32 sent = g_assistant.tcp.send_data(buff, length);
    if (sent != length)
    {
        g_assistant.connected = false;
        std::printf("assistant v2 send failed, disconnect\r\n");
    }
    return sent;
}

static void configure_packets(void)
{
    seekfree_assistant_v2_set_transfer(assistant_v2_transfer);
    seekfree_assistant_v2_camera_config(
        &g_assistant.camera,
        SEEKFREE_ASSISTANT_V2_CAMERA_TYPE_GRAY, // 明确灰度（与 MT9V03X 枚举值相同=2）
        static_cast<uint16>(k_img_w),  // 94
        static_cast<uint16>(k_img_h),  // 60
        g_assistant.send_image ? g_assistant.image_buffer[0] : nullptr);
    seekfree_assistant_v2_camera_boundary_config(
        &g_assistant.left_boundary,
        SEEKFREE_ASSISTANT_V2_DATA_TYPE_UINT8,
        k_color_left,
        static_cast<uint16>(k_img_h),
        g_assistant.left_pts);
    seekfree_assistant_v2_camera_boundary_config(
        &g_assistant.mid_boundary,
        SEEKFREE_ASSISTANT_V2_DATA_TYPE_UINT8,
        k_color_mid,
        static_cast<uint16>(k_img_h),
        g_assistant.mid_pts);
    seekfree_assistant_v2_camera_boundary_config(
        &g_assistant.right_boundary,
        SEEKFREE_ASSISTANT_V2_DATA_TYPE_UINT8,
        k_color_right,
        static_cast<uint16>(k_img_h),
        g_assistant.right_pts);
}

} // namespace

int assistant_v2_init(void)
{
    std::memset(g_assistant.image_buffer, 0, sizeof(g_assistant.image_buffer));
    std::memset(g_assistant.left_pts, 0, sizeof(g_assistant.left_pts));
    std::memset(g_assistant.mid_pts, 0, sizeof(g_assistant.mid_pts));
    std::memset(g_assistant.right_pts, 0, sizeof(g_assistant.right_pts));

    g_assistant.connected = false;
    // 默认发灰度图（任务：看 94x60 图 + 三线）；可用 ASSISTANT_SEND_IMAGE=0 只发线
    g_assistant.send_image = read_env_flag("ASSISTANT_SEND_IMAGE", true);
    if (std::getenv("CAR_ROUND_ASSISTANT_SEND_IMAGE") != nullptr)
        g_assistant.send_image = read_env_flag("CAR_ROUND_ASSISTANT_SEND_IMAGE", true);

    g_assistant.frame_div = static_cast<uint32>(
        read_env_int_clamped("ASSISTANT_DIV",
                             read_env_int_clamped("CAR_ROUND_ASSISTANT_DIV", k_default_frame_div, 1, 10000),
                             1,
                             10000));
    g_assistant.ip = k_default_ip;
    g_assistant.port = k_default_port;
    g_assistant.send_retry = static_cast<uint8>(
        read_env_int_clamped("ASSISTANT_SEND_RETRY", k_default_send_retry, 0, 20));
    g_assistant.retry_interval_ms = static_cast<uint16>(
        read_env_int_clamped("ASSISTANT_RETRY_MS", k_default_retry_interval_ms, 0, 1000));

    g_assistant.tcp.set_retry_param(g_assistant.send_retry, g_assistant.retry_interval_ms);
    if (g_assistant.tcp.init(g_assistant.ip, g_assistant.port) != 0)
    {
        std::printf("assistant v2 tcp error: %s:%u (control continues)\r\n",
                    g_assistant.ip, g_assistant.port);
        return -1;
    }

    g_assistant.connected = true;
    configure_packets();
    std::printf("assistant v2 tcp ok: %s:%u, size=%dx%d, div=%u, send_image=%d\r\n",
                g_assistant.ip,
                g_assistant.port,
                k_img_w,
                k_img_h,
                g_assistant.frame_div,
                g_assistant.send_image ? 1 : 0);
    return 0;
}

void assistant_v2_tick(uint32 frame_id)
{
    if (!g_assistant.connected)
        return;
    if ((frame_id % g_assistant.frame_div) != 0U)
        return;

    // 同帧：先更新边线，再拷图，再按 Car_Round 顺序发（线后图 或 图后线）
    update_boundaries();
    if (g_assistant.send_image)
        copy_gray_image();
    else
    {
        seekfree_assistant_v2_camera_send(&g_assistant.camera);
        if (!g_assistant.connected)
            return;
    }

    seekfree_assistant_v2_camera_boundary_send(&g_assistant.left_boundary);
    if (!g_assistant.connected)
        return;
    seekfree_assistant_v2_camera_boundary_send(&g_assistant.mid_boundary);
    if (!g_assistant.connected)
        return;
    seekfree_assistant_v2_camera_boundary_send(&g_assistant.right_boundary);
    if (!g_assistant.connected)
        return;

    if (g_assistant.send_image)
        seekfree_assistant_v2_camera_send(&g_assistant.camera);
}

void assistant_v2_close(void)
{
    g_assistant.connected = false;
}
