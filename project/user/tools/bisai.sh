#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_TOOL="${BISAI_REMOTE_TOOL:-$SCRIPT_DIR/car_remote_tool.sh}"

die() {
  echo "错误：$*" >&2
  exit 1
}

run_remote() {
  [[ -f "$REMOTE_TOOL" ]] || die "找不到 $REMOTE_TOOL"
  bash "$REMOTE_TOOL" "$@"
}

show_help() {
  echo "用法: $0 [menu|params|start|stop|apply|pull|result|log]"
}

show_board_params() {
  echo "==================== 板端现有参数 ===================="
  if ! run_remote params-summary; then
    die "无法读取板端现有参数，已阻止比赛操作"
  fi
  echo "======================================================"
}

show_menu() {
  echo "==================== 比赛工具 ===================="
  echo "1) 开始比赛（TRACK mode 10）"
  echo "2) 立即停车"
  echo "3) 写入本地参数到板端"
  echo "4) 从板端读取参数到本地"
  echo "5) 查看本场识别结果"
  echo "6) 查看车端日志"
  echo "q) 退出"
  echo "=================================================="
}

competition_menu() {
  local choice
  while true; do
    show_board_params
    show_menu
    if ! read -r -p "请选择操作: " choice; then
      return 0
    fi
    case "$choice" in
      1) run_remote start ;;
      2) run_remote stop ;;
      3) run_remote apply ;;
      4) run_remote pull ;;
      5) run_remote race-result ;;
      6) run_remote log ;;
      q|Q) return 0 ;;
      *) echo "无效选择：$choice" ;;
    esac
    echo
  done
}

case "${1:-menu}" in
  menu) competition_menu ;;
  params|params-summary|summary) show_board_params ;;
  start) show_board_params; run_remote start ;;
  stop) run_remote stop ;;
  apply) show_board_params; run_remote apply ;;
  pull) run_remote pull ;;
  result|race-result|race_result) run_remote race-result ;;
  log) run_remote log ;;
  -h|--help|help) show_help ;;
  *) die "未知命令：$1" ;;
esac
