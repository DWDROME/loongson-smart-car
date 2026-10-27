#ifndef ASSISTANT_V2_HPP_
#define ASSISTANT_V2_HPP_

#include "zf_common_typedef.hpp"

// 原因：迁自 Car_Round/code/AssistantV2；控制尺度 94x60 + 同帧三线。
// 失败只打日志，不改变控制输出（任务 R9）。
int assistant_v2_init(void);
void assistant_v2_tick(uint32 frame_id);
void assistant_v2_close(void);

#endif
