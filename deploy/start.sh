#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RUNTIME_DIR="$SCRIPT_DIR/.runtime"
RK_HOST="root@192.168.100.200"
REMOTE_PROJECT="/root/stereo_chessboard_calibrator"
REMOTE_STATUS="http://127.0.0.1:8765/api/status"
LOCAL_STATUS="http://127.0.0.1:18765/api/status"
LOCAL_URL="http://127.0.0.1:18765/"
FORWARD="127.0.0.1:18765:127.0.0.1:8765"
TUNNEL_PID="$RUNTIME_DIR/tunnel.pid"
TUNNEL_LOG="$RUNTIME_DIR/tunnel.log"
REMOTE_LOG="/tmp/world_intelligent_calibrate.log"

mkdir -p "$RUNTIME_DIR"

remote_status() {
  ssh -o BatchMode=yes -o ConnectTimeout=8 "$RK_HOST" \
    "curl -fsS '$REMOTE_STATUS' | grep -q '\"state\"'"
}

local_status() {
  curl --connect-timeout 2 -fsS "$LOCAL_STATUS" | grep -q '"state"'
}

wait_for_remote() {
  count=0
  while [ "$count" -lt 15 ]; do
    if remote_status; then
      return 0
    fi
    count=$((count + 1))
    sleep 1
  done
  return 1
}

wait_for_local() {
  count=0
  while [ "$count" -lt 10 ]; do
    if local_status; then
      return 0
    fi
    count=$((count + 1))
    sleep 1
  done
  return 1
}

echo "[1/3] 检查 RK3588 SSH..."
ssh -o BatchMode=yes -o ConnectTimeout=8 "$RK_HOST" true

echo "[2/3] 检查 RK3588 标定服务..."
if remote_status; then
  echo "      远端服务已运行，直接复用。"
else
  echo "      远端服务未运行，正在启动..."
  ssh -o BatchMode=yes -o ConnectTimeout=8 "$RK_HOST" \
    "cd '$REMOTE_PROJECT' && setsid -f ./calibrate --web --device /dev/video0 --square-mm 20 --target 32 --host 0.0.0.0 --port 8765 >'$REMOTE_LOG' 2>&1 </dev/null"
  if ! wait_for_remote; then
    echo "错误：RK3588 标定服务启动失败，请查看 $REMOTE_LOG" >&2
    exit 1
  fi
  echo "      远端服务已启动。"
fi

echo "[3/3] 检查 Codex 本地隧道..."
if local_status; then
  existing_pid=$(lsof -nP -t -iTCP:18765 -sTCP:LISTEN 2>/dev/null | head -n 1 || true)
  if [ -n "$existing_pid" ]; then
    printf '%s\n' "$existing_pid" >"$TUNNEL_PID"
  fi
  echo "      本地隧道已运行，直接复用。"
else
  occupied_pid=$(lsof -nP -t -iTCP:18765 -sTCP:LISTEN 2>/dev/null | head -n 1 || true)
  if [ -n "$occupied_pid" ]; then
    echo "错误：本机 18765 端口已被非标定服务占用（PID $occupied_pid）。" >&2
    exit 1
  fi
  ssh -f -N \
    -o BatchMode=yes \
    -o ConnectTimeout=8 \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=15 \
    -o ServerAliveCountMax=3 \
    -L "$FORWARD" \
    "$RK_HOST" 2>>"$TUNNEL_LOG"
  tunnel_pid=$(lsof -nP -t -iTCP:18765 -sTCP:LISTEN 2>/dev/null | head -n 1 || true)
  if [ -n "$tunnel_pid" ]; then
    printf '%s\n' "$tunnel_pid" >"$TUNNEL_PID"
  fi
  if ! wait_for_local; then
    echo "错误：SSH 隧道已启动，但本地网页健康检查失败。" >&2
    exit 1
  fi
  echo "      本地隧道已启动。"
fi

echo "启动完成：$LOCAL_URL"
