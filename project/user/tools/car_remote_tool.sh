#!/usr/bin/env bash
# 结构照抄 Car_Round/user/tools/car_remote_tool.sh，仅改二进制/参数键/网络。
# 改参数：编辑 tools/params_config.local.txt → apply（菜单1）→ 再 start 重启加载。
# 车端唯一真相：/root/params_config.txt（= REMOTE_PARAM_FILE；程序 PARAM_FILE_NAME 同路径）。
# 本地真相：本脚本旁 params_config.local.txt（= LOCAL_PARAM_FILE）。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UPPER_HOST_CONFIG="$SCRIPT_DIR/../../code/upper_host_config.h"
DEFAULT_PC_IP="$(sed -n 's/^#define UPPER_HOST_IP "\([^"]*\)"$/\1/p' "$UPPER_HOST_CONFIG")"
DEFAULT_VOFA_PORT="$(sed -n 's/^#define UPPER_HOST_PORT \([0-9][0-9]*\)$/\1/p' "$UPPER_HOST_CONFIG")"
if [[ -z "$DEFAULT_PC_IP" || -z "$DEFAULT_VOFA_PORT" ]]; then
  echo "错误：无法从 $UPPER_HOST_CONFIG 读取上位机默认配置" >&2
  exit 1
fi

SSH_TARGET="root@10.156.129.100"
REMOTE_PARAM_FILE="${REMOTE_PARAM_FILE:-/root/params_config.txt}"
REMOTE_APP_DIR="${REMOTE_APP_DIR:-/root}"
REMOTE_APP_NAME="${REMOTE_APP_NAME:-project}"
REMOTE_LOG_FILE="${REMOTE_LOG_FILE:-/tmp/project.log}"
REMOTE_RACE_RESULT_FILE="${REMOTE_RACE_RESULT_FILE:-/root/race_recognition_result.log}"
RUN_MENU_FLAG="${RUN_MENU_FLAG:-10}"
SPEED_TUNE_MENU_FLAG="${SPEED_TUNE_MENU_FLAG:-1}"
SENSOR_CAL_MENU_FLAG="${SENSOR_CAL_MENU_FLAG:-2}"
MIDDLE_TUNE_MENU_FLAG="${MIDDLE_TUNE_MENU_FLAG:-3}"
IMAGE_DEBUG_MENU_FLAG="${IMAGE_DEBUG_MENU_FLAG:-30}"
CAMERA_FOCUS_MENU_FLAG="${CAMERA_FOCUS_MENU_FLAG:-31}"
PC_IP="$DEFAULT_PC_IP"
VOFA_PORT="$DEFAULT_VOFA_PORT"
SSH_CONNECT_TIMEOUT="${SSH_CONNECT_TIMEOUT:-3}"
MENU_REFRESH_DELAY="${MENU_REFRESH_DELAY:-0}"
LOCAL_PARAM_FILE="${LOCAL_PARAM_FILE:-$SCRIPT_DIR/params_config.local.txt}"
LOCAL_RACE_RESULT_DIR="${LOCAL_RACE_RESULT_DIR:-$SCRIPT_DIR/race_results}"
LOCAL_MODEL_DIR="${LOCAL_MODEL_DIR:-$SCRIPT_DIR/../../model}"
# 模型切换只在这组文件/SHA与 LQ_NCNN 合同中同步；不保留自动 fallback。
LOCAL_NCNN_PARAM="${LOCAL_NCNN_PARAM:-$LOCAL_MODEL_DIR/tiny_classifier_fp32.ncnn.param}"
LOCAL_NCNN_BIN="${LOCAL_NCNN_BIN:-$LOCAL_MODEL_DIR/tiny_classifier_fp32.ncnn.bin}"
NCNN_PARAM_SHA256="f1e66346b3def63cc9fe15846241aaa8f84c59e82797dfe605a96dafde1b5cbd"
NCNN_BIN_SHA256="0c512b1bc0b8fa54baf6ed04f8897473253e7aee447b11b6366af94a1b90acc6"

PARAM_KEYS=(
  speed_kf_l speed_kp_l speed_ki_l
  speed_kf_r speed_kp_r speed_ki_r
  speed_max_i speed_max_out speed_max_error
  middle_kp middle_kd middle_ki middle_max_out middle_ref_slew middle_tpm_weight
  image_kp_scale image_kd threshold_static camera_exposure
  base_speed straight_trim picture_speed ramp_speed
  picture_recog_k picture_recog_base picture_near_dist
  picture_side_offset picture_hold_dist picture_finish_dist picture_conf_min
  red_brick_enable picture_roi_enable red_route_white_ratio red_route_confirm_frames
  red_brick_side_deadband red_brick_slow_speed red_brick_speed
  red_brick_offset_out red_brick_offset_back
  red_brick_slowdown_ms red_brick_out_ms red_brick_past_ms
  red_brick_back_ms red_brick_recover_ms
  forward_base huandao_l_state5_exit_yaw huandao_r_state5_exit_yaw middle_test_target
  count_per_meter count_per_meter_l count_per_meter_r track_width_m c_gyro
  small_err picture_err
  debug_rgb_r_min debug_rgb_rb_diff debug_rgb_rg_diff
)
RED_PARAM_KEYS=(
  red_brick_enable picture_roi_enable red_route_white_ratio red_route_confirm_frames
  red_brick_side_deadband red_brick_slow_speed red_brick_speed
  red_brick_offset_out red_brick_offset_back
  red_brick_slowdown_ms red_brick_out_ms red_brick_past_ms
  red_brick_back_ms red_brick_recover_ms
)

die() { echo "错误：$*" >&2; exit 1; }
pause() { sleep "$MENU_REFRESH_DELAY"; }
escape_sed_regex() { printf '%s' "$1" | sed 's/[.[\*^$()+?{}|\\]/\\&/g'; }
escape_sed_replacement() { printf '%s' "$1" | sed 's/[\/&\\]/\\&/g'; }
is_known_key() {
  local key="$1" k
  for k in "${PARAM_KEYS[@]}"; do [[ "$key" == "$k" ]] && return 0; done
  return 1
}

is_red_key() {
  local key="$1" k
  for k in "${RED_PARAM_KEYS[@]}"; do [[ "$key" == "$k" ]] && return 0; done
  return 1
}

float_in_range() {
  local value="$1" min="$2" max="$3" min_open="${4:-0}"
  awk -v v="$value" -v lo="$min" -v hi="$max" -v open="$min_open" \
    'BEGIN { exit !((open ? v > lo : v >= lo) && v <= hi) }'
}

validate_red_value() {
  local key="$1" value="$2"
  is_red_key "$key" || die "不允许修改非红砖参数：$key"
  [[ "$value" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || die "$key 必须是有限数字"
  case "$key" in
    red_brick_enable|picture_roi_enable)
      [[ "$value" == "0" || "$value" == "1" ]] || die "$key 必须是整数 0 或 1"
      ;;
    red_route_white_ratio)
      float_in_range "$value" 0 1 || die "$key 必须在 [0,1]"
      ;;
    red_route_confirm_frames)
      [[ "$value" =~ ^[0-9]+$ ]] || die "$key 必须是整数"
      (( value >= 1 && value <= 10 )) || die "$key 必须在 [1,10]"
      ;;
    red_brick_side_deadband)
      [[ "$value" =~ ^[0-9]+$ ]] || die "$key 必须是整数"
      (( value >= 0 && value <= 30 )) || die "$key 必须在 [0,30]"
      ;;
    red_brick_slow_speed|red_brick_speed)
      float_in_range "$value" 0 2000 1 || die "$key 必须在 (0,2000]"
      ;;
    red_brick_offset_out|red_brick_offset_back)
      float_in_range "$value" 0 40 || die "$key 必须在 [0,40]"
      ;;
    red_brick_slowdown_ms|red_brick_out_ms|red_brick_past_ms|red_brick_back_ms|red_brick_recover_ms)
      [[ "$value" =~ ^[0-9]+$ ]] || die "$key 必须是整数毫秒"
      (( value >= 1 && value <= 10000 )) || die "$key 必须在 [1,10000] ms"
      ;;
  esac
}

verify_sha256() {
  local file="$1" expected="$2" actual
  actual="$(sha256sum "$file" | awk '{print $1}')" || die "无法计算 SHA-256：$file"
  [[ "$actual" == "$expected" ]] || die "模型 SHA-256 不匹配：$file"
}

read_local_params() {
  [[ -f "$LOCAL_PARAM_FILE" ]] || die "找不到本地参数文件：$LOCAL_PARAM_FILE"
  local line key value
  local -A values=()
  while IFS= read -r line || [[ -n "$line" ]]; do
    line="${line%%#*}"; line="${line//$'\r'/}"
    [[ "$line" =~ ^[[:space:]]*$ ]] && continue
    if [[ "$line" == *"="* ]]; then
      key="${line%%=*}"; value="${line#*=}"
    else
      key="${line%%[:=]*}"; value="${line#*[:=]}"
    fi
    key="$(printf '%s' "$key" | xargs)"
    value="$(printf '%s' "$value" | xargs)"
    is_known_key "$key" || die "本地参数文件包含未知参数：$key"
    [[ "$value" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || die "参数 $key 的值不是数字：$value"
    [[ -z "${values[$key]+x}" ]] || die "本地参数重复：$key"
    if is_red_key "$key"; then
      validate_red_value "$key" "$value"
    fi
    values["$key"]="$value"
  done < "$LOCAL_PARAM_FILE"
  [[ -n "${values[base_speed]:-}" ]] || die "缺少 base_speed"
  local k
  for k in "${PARAM_KEYS[@]}"; do
    [[ -n "${values[$k]:-}" ]] || die "缺少参数：$k"
    printf '%s=%s\n' "$k" "${values[$k]}"
  done
}

ssh_car() {
  ssh -o "ConnectTimeout=$SSH_CONNECT_TIMEOUT" -o StrictHostKeyChecking=accept-new "$SSH_TARGET" "$@"
}

fetch_remote_params() {
  ssh_car "cat '$REMOTE_PARAM_FILE'" 2>/dev/null || true
}

show_remote_param_summary() {
  echo "当前车端关键参数："
  local remote_params key line value
  remote_params="$(fetch_remote_params)"
  if [[ -z "$remote_params" ]]; then
    echo "  无法读取车端参数。请检查 SSH/路径。"
    return 1
  fi
  echo "速度预设值（状态命中后覆盖 base_speed）："
  for key in base_speed straight_trim picture_speed ramp_speed camera_exposure speed_kf_l speed_kp_l speed_ki_l speed_kf_r speed_kp_r speed_ki_r middle_kp middle_ki middle_kd middle_max_out middle_ref_slew middle_tpm_weight huandao_l_state5_exit_yaw huandao_r_state5_exit_yaw count_per_meter count_per_meter_l count_per_meter_r c_gyro; do
    line="$(printf '%s\n' "$remote_params" | awk -F= -v k="$key" '$1==k{print;exit}')"
    if [[ -n "$line" ]]; then
      value="${line#*=}"
      printf '  %-28s %s\n' "$key:" "$value"
    else
      printf '  %-28s %s\n' "$key:" "未找到"
    fi
  done
  echo "平面红纸/模型参数（physical ROI80 -> model 64x64）："
  for key in picture_recog_k picture_recog_base picture_near_dist picture_side_offset picture_hold_dist picture_finish_dist picture_conf_min; do
    line="$(printf '%s\n' "$remote_params" | awk -F= -v k="$key" '$1==k{print;exit}')"
    if [[ -n "$line" ]]; then
      printf '  %-28s %s\n' "$key:" "${line#*=}"
    else
      printf '  %-28s %s\n' "$key:" "未找到（先执行菜单 1）"
    fi
  done
  echo "立体红砖 detour 参数："
  for key in "${RED_PARAM_KEYS[@]}"; do
    line="$(printf '%s\n' "$remote_params" | awk -F= -v k="$key" '$1==k{print;exit}')"
    if [[ -n "$line" ]]; then
      printf '  %-28s %s\n' "$key:" "${line#*=}"
    else
      printf '  %-28s %s\n' "$key:" "未找到（先执行菜单 1）"
    fi
  done
}

show_status() {
  clear
  echo "==================== 远程工具（照 Car_Round） ===================="
  echo "目标：$SSH_TARGET"
  echo "车端参数：$REMOTE_PARAM_FILE"
  echo "本地参数：$LOCAL_PARAM_FILE"
  echo "上位机：$PC_IP:$VOFA_PORT"
  echo "------------------------------------------------------------"
  show_remote_param_summary || true
  echo "------------------------------------------------------------"
  echo "1) 写入本地参数到车端"
  echo "2) 停车"
  echo "3) 跑线：逐飞助手画面（mode=10，assistant）"
  echo "4) 跑线：无上位机（mode=10，none）"
  echo "5) 跑线：VOFA（mode=10，vofa）"
  echo "6) 调速度环（mode=1，VOFA；base=0 仍零速闭环）"
  echo "7) 调中环（mode=3，VOFA）"
  echo "8) 锁电机看图：逐飞（mode=30，assistant）"
  echo "9) 锁电机无输出（mode=30，none）"
  echo "10) 传感器标定（mode=2，none）"
  echo "11) 查看车端完整参数"
  echo "12) 查看日志"
  echo "13) 回读参数到本地"
  echo "14) 上传本地二进制 + NCNN param/bin"
  echo "15) 图片识别调试（mode=30，锁电机 + 识别日志）"
  echo "16) 镜头清晰度+识别调试（mode=31，彩色320x240，HTTP约5FPS）"
  echo "17) 设置相机曝光并重启清晰度模式（-1=driver-default，0=auto，20..200=manual）"
  echo "18) 查看本场识别结果（第二斑马线停车后抓取）"
  echo "19) 单独设置红砖参数并重启 mode=31（推荐）"
  echo "q) 退出"
  echo "============================================================"
  echo "速度预设：base_speed=常规，picture_speed=picture==2，ramp_speed=ramp==2"
  echo "平面红纸：recog 开扫 | near 近检 | side 贴边 | hold/finish 里程 | conf 门槛"
  echo "红砖参数：使用 red-set <key> <value>，只改一个键并自动锁电机重启"
  echo "修改本地 txt → 1 上传 → 再 3/4/5 重启加载"
}

apply_params() {
  # 整文件覆盖到 REMOTE_PARAM_FILE（与程序 PARAM_FILE_NAME=/root/params_config.txt 一致）
  # 不再 sed 逐项改：避免与 /params_config.txt 或半截文件分叉。
  local params tmp
  params="$(read_local_params)"
  [[ -n "$params" ]] || die "本地参数为空"
  tmp="$(mktemp)"
  printf '%s\n' "$params" >"$tmp"
  scp -o "ConnectTimeout=$SSH_CONNECT_TIMEOUT" -O "$tmp" "$SSH_TARGET:$REMOTE_PARAM_FILE"
  rm -f "$tmp"
  # 清理历史误用的 cwd=/ 假文件，避免人工对照时混淆
  ssh_car "rm -f /params_config.txt 2>/dev/null || true; ls -la '$REMOTE_PARAM_FILE'"
  echo "已整文件写入本地参数 -> 车端 $REMOTE_PARAM_FILE"
  echo "程序固定读取：/root/params_config.txt（与 REMOTE_PARAM_FILE 一致）"
}

stop_app() {
  ssh_car "killall -9 '$REMOTE_APP_NAME' 2>/dev/null || true"
  echo "已停车：$REMOTE_APP_NAME 已停止。"
}

start_camera_focus() {
  local host="${SSH_TARGET#*@}"
  start_app "$CAMERA_FOCUS_MENU_FLAG" none "镜头清晰度调试"
  echo "浏览器打开：http://$host:8080"
  echo "画面顶部为完整 320x240 BGR 彩色图；底部显示 Laplacian 清晰度、亮度和图传 FPS。"
  echo "同一状态条还显示当前帧模型类别、置信度和图像行距表估算距离。"
  echo "固定物体、距离和曝光模式后缓慢拧焦，同时观察亮度，以清晰度稳定峰值为准。"
}

set_camera_exposure() {
  local value="${1:-}"
  [[ -n "$value" ]] || read -r -p "camera_exposure（-1=driver-default，0=auto，20..200=manual）： " value
  [[ "$value" =~ ^-?[0-9]+$ ]] || die "camera_exposure 必须是整数"
  if (( value != -1 && value != 0 && (value < 20 || value > 200) )); then
    die "camera_exposure 必须为 -1、0 或 20..200"
  fi
  grep -q '^camera_exposure=' "$LOCAL_PARAM_FILE" || die "本地参数缺少 camera_exposure"
  ssh_car "grep -q '^camera_exposure=' '$REMOTE_PARAM_FILE' || exit 2; \
    stamp=\$(date +%Y%m%d_%H%M%S); \
    cp '$REMOTE_PARAM_FILE' '${REMOTE_PARAM_FILE}.before_exposure_'\${stamp}; \
    sed -i 's/^camera_exposure=.*/camera_exposure=$value/' '$REMOTE_PARAM_FILE'; \
    grep -n '^camera_exposure=' '$REMOTE_PARAM_FILE'"
  sed -i "s/^camera_exposure=.*/camera_exposure=$value/" "$LOCAL_PARAM_FILE"
  echo "已同步 camera_exposure=$value；其他参数未覆盖。"
  start_camera_focus
}

red_set() {
  local key="${1:-}" value="${2:-}"
  [[ -n "$key" ]] || read -r -p "红砖参数 key： " key
  [[ -n "$value" ]] || read -r -p "$key 新值： " value
  validate_red_value "$key" "$value"
  [[ -f "$LOCAL_PARAM_FILE" ]] || die "找不到本地参数文件：$LOCAL_PARAM_FILE"
  read_local_params >/dev/null

  local local_count
  local_count="$(awk -F= -v k="$key" '$1==k{n++} END{print n+0}' "$LOCAL_PARAM_FILE")"
  [[ "$local_count" == "1" ]] || die "本地参数 $key 必须且只能出现一次，当前 $local_count 次"

  if [[ "${RED_SET_DRY_RUN:-0}" == "1" ]]; then
    echo "DRY-RUN: $key=$value 校验通过；不会连接或修改车端。"
    return 0
  fi

  local local_backup local_tmp remote_backup remote_value
  local_backup="$(mktemp "${LOCAL_PARAM_FILE}.before_red_set_XXXXXX")"
  cp "$LOCAL_PARAM_FILE" "$local_backup"

  remote_backup="$(ssh_car "set -eu; \
    count=\$(awk -F= -v k='$key' '\$1==k{n++} END{print n+0}' '$REMOTE_PARAM_FILE'); \
    [ \"\$count\" -eq 1 ] || { echo 'remote key count invalid' >&2; exit 2; }; \
    backup=\$(mktemp '${REMOTE_PARAM_FILE}.before_red_set_XXXXXX'); \
    cp '$REMOTE_PARAM_FILE' \"\$backup\"; \
    tmp=\$(mktemp '${REMOTE_PARAM_FILE}.tmp.XXXXXX'); \
    awk -F= -v k='$key' -v v='$value' '{if (\$1==k) print k \"=\" v; else print}' \
      '$REMOTE_PARAM_FILE' >\"\$tmp\"; \
    mv \"\$tmp\" '$REMOTE_PARAM_FILE'; \
    actual=\$(awk -F= -v k='$key' '\$1==k{print substr(\$0,index(\$0,\"=\")+1); exit}' '$REMOTE_PARAM_FILE'); \
    [ \"\$actual\" = '$value' ] || { cp \"\$backup\" '$REMOTE_PARAM_FILE'; exit 3; }; \
    printf '%s' \"\$backup\"")" || die "车端单参数更新失败；本地未修改"

  local_tmp="$(mktemp "${LOCAL_PARAM_FILE}.tmp.XXXXXX")"
  if ! awk -F= -v k="$key" -v v="$value" \
      '{if ($1==k) print k "=" v; else print}' "$LOCAL_PARAM_FILE" >"$local_tmp" ||
     ! mv "$local_tmp" "$LOCAL_PARAM_FILE"; then
    rm -f "$local_tmp"
    ssh_car "cp '$remote_backup' '$REMOTE_PARAM_FILE'" || true
    die "本地参数更新失败；已请求恢复车端备份"
  fi

  if ! remote_value="$(ssh_car "awk -F= -v k='$key' '\$1==k{print substr(\$0,index(\$0,\"=\")+1); exit}' '$REMOTE_PARAM_FILE'")"; then
    cp "$local_backup" "$LOCAL_PARAM_FILE"
    ssh_car "cp '$remote_backup' '$REMOTE_PARAM_FILE'" || true
    die "车端回读失败；已请求恢复本地和车端备份"
  fi
  if [[ "$remote_value" != "$value" ]] ||
     [[ "$(awk -F= -v k="$key" '$1==k{print substr($0,index($0,"=")+1); exit}' "$LOCAL_PARAM_FILE")" != "$value" ]]; then
    cp "$local_backup" "$LOCAL_PARAM_FILE"
    ssh_car "cp '$remote_backup' '$REMOTE_PARAM_FILE'" || true
    die "单参数回读不一致；已请求恢复本地和车端备份"
  fi

  echo "已同步 $key=$value；未改动其他参数。"
  echo "本地备份：$local_backup"
  echo "车端备份：$remote_backup"
  start_camera_focus
}

# output: none | vofa | assistant
# 第 4 参数可选 freewheel：mode1 手推 cpm（PWM0）；默认不传=速度环闭环（base=0 为零速 PI）
start_app() {
  local menu_flag="$1"
  local output_mode="$2"
  local mode_name="$3"
  local freewheel="${4:-}"
  local out_arg="--none" fw_env="0" fw_arg=""
  case "$output_mode" in
    vofa) out_arg="--vofa" ;;
    assistant) out_arg="--assistant" ;;
  esac
  if [[ "$freewheel" == "freewheel" ]]; then
    fw_env="1"
    fw_arg="--freewheel"
  fi
  # 截断日志：避免 kill 后残留刷屏把 /tmp 撑满（曾见 450MB+）
  ssh_car "killall -9 '$REMOTE_APP_NAME' 2>/dev/null || true; : > '$REMOTE_LOG_FILE'; cd '$REMOTE_APP_DIR'; \
    RUN_MODE='$menu_flag' OUTPUT_MODE='$output_mode' SPEED_FREEWHEEL='$fw_env' \
    ASSISTANT_DIV=5 ASSISTANT_SEND_IMAGE=1 \
    nohup ./'$REMOTE_APP_NAME' '$menu_flag' $out_arg $fw_arg >'$REMOTE_LOG_FILE' 2>&1 &"
  echo "已启动 $REMOTE_APP_NAME $mode_name，mode=$menu_flag，output=$output_mode，freewheel=$fw_env，日志：$REMOTE_LOG_FILE"
}

show_full_params() {
  echo "车端完整参数："; echo "------------------------------------------------------------"
  ssh_car "cat '$REMOTE_PARAM_FILE' 2>/dev/null || echo 暂无"
}

show_app_log() {
  echo "日志：$REMOTE_LOG_FILE"; echo "------------------------------------------------------------"
  ssh_car "tail -80 '$REMOTE_LOG_FILE' 2>/dev/null || echo 暂无"
}

show_recognition_log() {
  echo "最近图片识别日志：$REMOTE_LOG_FILE"
  echo "------------------------------------------------------------"
  ssh_car "touch '$REMOTE_LOG_FILE'; tail -n 200 '$REMOTE_LOG_FILE' | awk '/\\[REC\\]|NCNN|ROI|out0|camera|ImageDeal|vision init|failed|error|失败|错误/ { print }'"
}

show_race_result() {
  local stamp local_file version status count overflow sequence
  local suffix=1
  local -a classes=()
  local item label display=""

  if ! ssh_car "test -f '$REMOTE_RACE_RESULT_FILE'"; then
    echo "本场识别结果不存在：请先以 TRACK 模式开始比赛。"
    return 0
  fi
  mkdir -p "$LOCAL_RACE_RESULT_DIR"
  stamp="$(date +%Y%m%d_%H%M%S)"
  local_file="$LOCAL_RACE_RESULT_DIR/race_result_${stamp}.log"
  while [[ -e "$local_file" ]]; do
    local_file="$LOCAL_RACE_RESULT_DIR/race_result_${stamp}_${suffix}.log"
    suffix=$((suffix + 1))
  done
  if ! scp -o "ConnectTimeout=$SSH_CONNECT_TIMEOUT" \
      -o StrictHostKeyChecking=accept-new -O \
      "$SSH_TARGET:$REMOTE_RACE_RESULT_FILE" "$local_file"; then
    rm -f "$local_file"
    echo "抓取本场识别结果失败。"
    return 0
  fi

  version="$(awk -F= '$1=="version"{print substr($0,index($0,"=")+1); exit}' "$local_file")"
  status="$(awk -F= '$1=="status"{print substr($0,index($0,"=")+1); exit}' "$local_file")"
  count="$(awk -F= '$1=="count"{print substr($0,index($0,"=")+1); exit}' "$local_file")"
  overflow="$(awk -F= '$1=="overflow"{print substr($0,index($0,"=")+1); exit}' "$local_file")"
  sequence="$(awk -F= '$1=="sequence"{print substr($0,index($0,"=")+1); exit}' "$local_file")"

  if [[ "$version" != "1" || ! "$count" =~ ^[0-9]+$ ||
        ( "$status" != "running" && "$status" != "finished" ) ||
        ( "$overflow" != "0" && "$overflow" != "1" ) ]]; then
    echo "结果文件格式错误，原始文件已保留：$local_file"
    return 0
  fi
  if (( count > 8 )); then
    echo "结果数量超过固定容量 8，原始文件已保留：$local_file"
    return 0
  fi

  if [[ -n "$sequence" ]]; then
    IFS=',' read -r -a classes <<< "$sequence"
  fi
  if (( ${#classes[@]} != count )); then
    echo "结果数量与顺序不一致，原始文件已保留：$local_file"
    return 0
  fi

  for item in "${classes[@]}"; do
    case "$item" in
      supplies) label="物资" ;;
      vehicle) label="车辆" ;;
      weapon) label="武器" ;;
      *)
        echo "结果中包含未知类别 '$item'，原始文件已保留：$local_file"
        return 0
        ;;
    esac
    if [[ -n "$display" ]]; then
      display+=" -> "
    fi
    display+="$label"
  done

  if [[ -z "$display" ]]; then
    display="无图片识别记录"
  fi
  if [[ "$status" == "running" ]]; then
    echo "本场识别结果（未到第二个斑马线）：$display"
  else
    echo "本场识别结果：$display"
  fi
  if [[ "$overflow" == "1" ]]; then
    echo "警告：识别次数超过 8 次，仅保留前 8 次。"
  fi
  echo "结果已保存：$local_file"
}

stream_recognition_log() {
  echo "实时图片识别日志：$REMOTE_LOG_FILE"
  echo "只显示 NCNN、[REC]、ROI、相机和错误；按回车结束查看，车端识别继续运行。"
  echo "------------------------------------------------------------"
  ssh -o "ConnectTimeout=$SSH_CONNECT_TIMEOUT" -o StrictHostKeyChecking=accept-new \
    "$SSH_TARGET" \
    "touch '$REMOTE_LOG_FILE'; tail -n 100 -f '$REMOTE_LOG_FILE' | awk '/\\[REC\\]|NCNN|ROI|out0|camera|ImageDeal|vision init|failed|error|失败|错误/ { print; fflush(); }'" &
  local tail_pid=$!
  read -r
  kill "$tail_pid" 2>/dev/null || true
  wait "$tail_pid" 2>/dev/null || true
}

image_debug_menu() {
  while true; do
    clear
    echo "==================== 图片识别调试 ===================="
    echo "模式：mode=$IMAGE_DEBUG_MENU_FLAG，PIT 每拍锁电机，作者单帧 NCNN 识别"
    echo "模型：$(basename "$LOCAL_NCNN_PARAM") / $(basename "$LOCAL_NCNN_BIN")"
    echo "日志：$REMOTE_LOG_FILE"
    echo "------------------------------------------------------------"
    echo "1) 启动纯识别（无上位机）并实时看日志"
    echo "2) 启动纯识别（逐飞助手画面）并实时看日志"
    echo "3) 实时查看识别日志"
    echo "4) 查看最近识别日志"
    echo "5) 停止程序"
    echo "b) 返回主菜单"
    echo "============================================================"
    read -r -p "请选择操作： " choice
    case "$choice" in
      1) start_app "$IMAGE_DEBUG_MENU_FLAG" none "锁电机纯识别"; stream_recognition_log ;;
      2) start_app "$IMAGE_DEBUG_MENU_FLAG" assistant "锁电机纯识别+逐飞"; stream_recognition_log ;;
      3) stream_recognition_log ;;
      4) show_recognition_log; read -r -p "按回车继续。" ;;
      5) stop_app; read -r -p "按回车继续。" ;;
      b|B) return ;;
      *) echo "无效选择：$choice"; pause ;;
    esac
  done
}

pull_params() {
  ssh_car "cat '$REMOTE_PARAM_FILE'" >"$LOCAL_PARAM_FILE" || die "回读失败"
  echo "已回读 -> $LOCAL_PARAM_FILE"
}

upload_bin() {
  local local_bin="${1:-}"
  [[ -n "$local_bin" ]] || read -r -p "本地 project 路径: " local_bin
  [[ -f "$local_bin" ]] || die "不存在 $local_bin"
  [[ -f "$LOCAL_NCNN_PARAM" ]] || die "不存在 $LOCAL_NCNN_PARAM"
  [[ -f "$LOCAL_NCNN_BIN" ]] || die "不存在 $LOCAL_NCNN_BIN"
  verify_sha256 "$LOCAL_NCNN_PARAM" "$NCNN_PARAM_SHA256"
  verify_sha256 "$LOCAL_NCNN_BIN" "$NCNN_BIN_SHA256"
  stop_app
  scp -o "ConnectTimeout=$SSH_CONNECT_TIMEOUT" -O "$local_bin" "$SSH_TARGET:$REMOTE_APP_DIR/$REMOTE_APP_NAME"
  scp -o "ConnectTimeout=$SSH_CONNECT_TIMEOUT" -O "$LOCAL_NCNN_PARAM" "$LOCAL_NCNN_BIN" "$SSH_TARGET:$REMOTE_APP_DIR/"
  ssh_car "cd '$REMOTE_APP_DIR' && printf '%s  %s\n%s  %s\n' \
    '$NCNN_PARAM_SHA256' '$(basename "$LOCAL_NCNN_PARAM")' \
    '$NCNN_BIN_SHA256' '$(basename "$LOCAL_NCNN_BIN")' | sha256sum -c -"
  ssh_car "chmod +x '$REMOTE_APP_DIR/$REMOTE_APP_NAME'"
  echo "主程序和 NCNN param/bin 上传完成。"
}

ensure_local() {
  [[ -f "$LOCAL_PARAM_FILE" ]] && return 0
  cat >"$LOCAL_PARAM_FILE" <<'EOF'
# 速度预设值：常规 / picture==2 / ramp==2；菜单 1 上传后重启程序
base_speed=0.0000
straight_trim=0.0250
picture_speed=100.0000
ramp_speed=150.0000
picture_recog_k=0.1000
picture_recog_base=40.0000
picture_near_dist=30.0000
picture_side_offset=10.0000
picture_hold_dist=30.0000
picture_finish_dist=40.0000
picture_conf_min=0.5000
red_brick_enable=0
picture_roi_enable=0
red_route_white_ratio=0.5500
red_route_confirm_frames=2
red_brick_side_deadband=5
red_brick_slow_speed=100.0000
red_brick_speed=220.0000
red_brick_offset_out=8.0000
red_brick_offset_back=10.0000
red_brick_slowdown_ms=333
red_brick_out_ms=1167
red_brick_past_ms=667
red_brick_back_ms=833
red_brick_recover_ms=500
speed_kf_l=8.0000
speed_kp_l=30.0000
speed_ki_l=0.0000
speed_kf_r=8.0000
speed_kp_r=30.0000
speed_ki_r=0.0000
speed_max_i=1500.0000
speed_max_out=5000.0000
speed_max_error=2000.0000
middle_kp=0.7000
middle_kd=0.0000
middle_ki=0.3000
middle_max_out=70.0000
middle_ref_slew=0.0000
middle_tpm_weight=0.0000
image_kp_scale=1.0000
image_kd=2.0000
threshold_static=35.0000
camera_exposure=-1
forward_base=30.0000
huandao_l_state5_exit_yaw=25.0000
huandao_r_state5_exit_yaw=-55.0000
middle_test_target=0.0000
count_per_meter=35000.0000
count_per_meter_l=0.0000
count_per_meter_r=0.0000
track_width_m=0.1500
c_gyro=28.1448
small_err=15
picture_err=30
debug_rgb_r_min=120
debug_rgb_rb_diff=35
debug_rgb_rg_diff=35
EOF
  echo "已生成 $LOCAL_PARAM_FILE （base_speed=0）"
}

main() {
  case "${1:-}" in
    params-summary|params_summary|summary) show_remote_param_summary; return ;;
  esac
  ensure_local
  case "${1:-}" in
    apply) apply_params; return ;;
    stop) stop_app; return ;;
    start|start-none) start_app "$RUN_MENU_FLAG" none "无上位机跑线"; return ;;
    start-vofa|vofa) start_app "$RUN_MENU_FLAG" vofa "VOFA跑线"; return ;;
    start-assistant|assistant) start_app "$RUN_MENU_FLAG" assistant "逐飞跑线"; return ;;
    tune-speed) start_app "$SPEED_TUNE_MENU_FLAG" vofa "速度环"; return ;;
    # 手推 cpm：必须 --freewheel（PWM0）；勿再把 base=0 当成松电机语义
    cal-1m|cal-cpm)
      start_app "$SPEED_TUNE_MENU_FLAG" none "手推cpm(freewheel)" freewheel
      return
      ;;
    tune-speed-none) start_app "$SPEED_TUNE_MENU_FLAG" none "速度环无上位机"; return ;;
    tune-middle) start_app "$MIDDLE_TUNE_MENU_FLAG" vofa "中环"; return ;;
    debug-image|debug-assistant) start_app "$IMAGE_DEBUG_MENU_FLAG" assistant "锁电机看图"; return ;;
    debug-image-none|debug-none) start_app "$IMAGE_DEBUG_MENU_FLAG" none "锁电机图像分类"; return ;;
    focus|camera-focus|focus-debug) start_camera_focus; return ;;
    exposure|camera-exposure) set_camera_exposure "${2:-}"; return ;;
    red-set|red_set) red_set "${2:-}" "${3:-}"; return ;;
    race-result|race_result) show_race_result; return ;;
    recognition-log|rec-log) show_recognition_log; return ;;
    recognition-follow|rec-follow) stream_recognition_log; return ;;
    recognition|image-debug-menu) image_debug_menu; return ;;
    show) show_full_params; return ;;
    log) show_app_log; return ;;
    pull) pull_params; return ;;
    upload) upload_bin "${2:-}"; return ;;
    ""|menu) ;;
    -h|--help|help)
      echo "用法: $0 [menu|apply|red-set KEY VALUE|stop|start|start-vofa|assistant|tune-speed|cal-1m|tune-middle|debug-assistant|debug-none|focus|exposure VALUE|race-result|recognition|recognition-log|recognition-follow|params-summary|show|log|pull|upload]"
      return
      ;;
    *) die "未知命令：$1" ;;
  esac

  while true; do
    show_status
    read -r -p "请选择操作： " choice
    case "$choice" in
      1) apply_params; pause ;;
      2) stop_app; pause ;;
      3) start_app 10 assistant "逐飞跑线"; pause ;;
      4) start_app 10 none "无上位机跑线"; pause ;;
      5) start_app 10 vofa "VOFA跑线"; pause ;;
      6) start_app 1 vofa "速度环"; pause ;;
      7) start_app 3 vofa "中环"; pause ;;
      8) start_app 30 assistant "锁电机看图"; pause ;;
      9) start_app 30 none "锁电机"; pause ;;
      10) start_app 2 none "传感器"; pause ;;
      11) show_full_params; pause ;;
      12) show_app_log; pause ;;
      13) pull_params; pause ;;
      14) upload_bin; pause ;;
      15) image_debug_menu ;;
      16) start_camera_focus; pause ;;
      17) set_camera_exposure; pause ;;
      18) show_race_result; pause ;;
      19) red_set; pause ;;
      q|Q) exit 0 ;;
      *) echo "无效选择：$choice"; pause ;;
    esac
  done
}

main "$@"
