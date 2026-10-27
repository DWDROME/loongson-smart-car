#ifndef VOFA_TELEMETRY_H
#define VOFA_TELEMETRY_H

#include <stdint.h>

// 原因：固定 48 通道 JustFloat；控制拍只写快照，主循环才发送（禁止 5ms 回调内 TCP）。
#define VOFA_CHANNEL_COUNT 48

extern float g_vofa_snapshot[VOFA_CHANNEL_COUNT];
extern volatile int g_vofa_snapshot_ready;

void Vofa_FillSnapshot(void);          // 可在控制拍末调用，仅 memcpy 到快照
void Vofa_InitTcp(const char *ip, int port);
void Vofa_TrySendFromMain(void);       // 主循环限频发送
void Vofa_Close(void);

// CSV 表头（与通道顺序一致），供 remote/日志
const char *Vofa_CsvHeader(void);

#endif
