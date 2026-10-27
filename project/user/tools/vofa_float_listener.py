#!/usr/bin/env python3
"""VOFA JustFloat TCP 监听：迁移自 Car_Round/user/tools/vofa_float_listener.py。

Car_Round 为 13 通道；当前工程固定 48 通道（见 vofa_telemetry / 合同文档）。
默认监听 0.0.0.0:8086，与本车 VOFA 端口一致。
"""

from __future__ import annotations

import argparse
import socket
import struct
import time
from contextlib import nullcontext
from pathlib import Path

TAIL = b"\x00\x00\x80\x7f"
CHANNEL_COUNT = 48

# 与 research/control-topology-and-telemetry-contract.md 顺序一致
NAMES_48 = [
    "control_tick",
    "control_dt_ms",
    "run_mode",
    "image_frame_id",
    "image_age_ms",
    "image_period_ms",
    "forward_row",
    "Dir_err",
    "Image_Kp_active",
    "Image_Kd_active",
    "Image_out",
    "middle_reference",
    "Tpm_Dis",
    "G_dis",
    "Dis_Speed",
    "middle_error",
    "middle_dP",
    "middle_dI",
    "Dis_Out",
    "middle_dU",
    "speed_goal",
    "left_target",
    "left_feedback",
    "left_error",
    "left_ff",
    "left_p",
    "left_i",
    "left_integral",
    "PWM_L",
    "right_target",
    "right_feedback",
    "right_error",
    "right_ff",
    "right_p",
    "right_i",
    "right_integral",
    "PWM_R",
    "saturation_bits",
    "shizi_state",
    "Huandao_L",
    "Huandao_R",
    "zebra_state",
    "picture_state",
    "small_rock_state",
    "ramp_state",
    "img_top",
    "jump_point",
    "dl1x_distance_raw",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description=f"Listen for JustFloat frames: {CHANNEL_COUNT} float32 + 00 00 80 7f."
    )
    parser.add_argument("--host", default="0.0.0.0", help="listen address")
    parser.add_argument("--port", type=int, default=8086, help="listen port")
    parser.add_argument(
        "--mode",
        choices=("full", "speed", "raw"),
        default="speed",
        help="full=48名; speed=左右速度子集; raw=ch0..",
    )
    parser.add_argument("--every", type=int, default=5, help="print every N frames")
    parser.add_argument(
        "--log",
        default="auto",
        help="CSV log path. Default: user/tools/vofa_logs/vofa_<mode>_<time>.csv",
    )
    parser.add_argument("--no-log", action="store_true", help="do not save CSV log")
    return parser.parse_args()


def names_for_mode(mode: str):
    if mode == "full":
        return NAMES_48
    if mode == "speed":
        # 只打印速度相关通道（索引仍对应整帧 48）
        return NAMES_48
    return [f"ch{i}" for i in range(CHANNEL_COUNT)]


def speed_print_indices():
    # speed_goal .. PWM_R
    return list(range(20, 37))


def print_frame(frame_id: int, values, names, mode: str):
    if mode == "speed":
        idxs = speed_print_indices()
        pairs = [f"{names[i]}={values[i]:.3f}" for i in idxs]
    else:
        pairs = [f"{n}={v:.3f}" for n, v in zip(names, values)]
    print(f"[{frame_id}] " + " ".join(pairs))


def main():
    args = parse_args()
    names = names_for_mode(args.mode)
    frame_bytes = CHANNEL_COUNT * 4 + 4

    log_path = None
    if not args.no_log:
        if args.log == "auto":
            log_dir = Path(__file__).resolve().parent / "vofa_logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"vofa_{args.mode}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        else:
            log_path = Path(args.log)
            log_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"listening {args.host}:{args.port} channels={CHANNEL_COUNT} mode={args.mode}")
    if log_path:
        print(f"csv log: {log_path}")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)

    frame_id = 0
    with (log_path.open("w", encoding="utf-8") if log_path else nullcontext()) as logf:
        if logf:
            logf.write("frame," + ",".join(NAMES_48) + "\n")

        while True:
            conn, addr = srv.accept()
            print(f"connected from {addr}")
            buf = b""
            try:
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while True:
                        idx = buf.find(TAIL)
                        if idx < 0:
                            if len(buf) > frame_bytes * 2:
                                buf = buf[-(frame_bytes - 1) :]
                            break
                        start = idx - CHANNEL_COUNT * 4
                        if start < 0:
                            buf = buf[idx + 4 :]
                            continue
                        payload = buf[start:idx]
                        buf = buf[idx + 4 :]
                        if len(payload) != CHANNEL_COUNT * 4:
                            continue
                        values = struct.unpack("<" + "f" * CHANNEL_COUNT, payload)
                        frame_id += 1
                        if frame_id % max(1, args.every) == 0:
                            print_frame(frame_id, values, names, args.mode)
                        if logf:
                            logf.write(
                                str(frame_id)
                                + ","
                                + ",".join(f"{v:.6f}" for v in values)
                                + "\n"
                            )
                            logf.flush()
            finally:
                conn.close()
                print("disconnected")


if __name__ == "__main__":
    main()
