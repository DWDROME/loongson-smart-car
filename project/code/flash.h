#ifndef FLASH_H
#define FLASH_H

// 绝对路径：与 car_remote_tool REMOTE_PARAM_FILE 一致，避免 cwd=/ 读到 /params_config.txt
#define PARAM_FILE_NAME "/root/params_config.txt"

// 原因：Flash 只保留视觉等仍在用的字段；控制参数直接写入 Velocity/Dis_1/Image，不再用 Flash.kp 冒充控制器。
struct FlashInformation
{
    // 遗留字段：不再接控制环；保留文件兼容，加载后忽略对 PID 的作用
    float kp;
    float ki;
    float kd;
    int small_err;
    int picture_err;
    int forward; // 若未提供 forward_base，可作回退
    int debug_rgb_r_min;
    int debug_rgb_rb_diff;
    int debug_rgb_rg_diff;
    int camera_exposure;
};
extern struct FlashInformation Flash;

namespace red_event
{
struct RedEventRouterConfig;
struct RedBrickDetourConfig;
}

void Param_Init(void);
void Param_SaveSingle(const char* key, float value);
void Param_SaveAll(void);
// 原因：把文件中的控制键写入实际 PID 结构（须在 Direction_PID_Init 之后调用）
void Param_ApplyToControllers(void);
int Param_IsApplyOk(void);
const red_event::RedEventRouterConfig &Param_RedEventRouterConfig(void);
const red_event::RedBrickDetourConfig &Param_RedBrickDetourConfig(void);

#define SaveInt(key, val)   Param_SaveSingle(key, (float)(val))
#define SaveFloat(key, val) Param_SaveSingle(key, (float)(val))

#endif
