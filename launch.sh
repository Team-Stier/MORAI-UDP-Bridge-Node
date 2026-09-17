#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_SHARE="$PACKAGE_DIR"
if command -v rospack >/dev/null 2>&1; then
    PACKAGE_SHARE="$(rospack find morai_udp_bridge)"
fi
CONFIG="${MORAI_BRIDGE_CONFIG:-${PACKAGE_SHARE}/config/config.yaml}"
ROSLAUNCH_ARGS=()

for arg in "$@"; do
    case "$arg" in
        config:=*) CONFIG="${arg#config:=}" ;;
        *) ROSLAUNCH_ARGS+=("$arg") ;;
    esac
done

HELPER="${PACKAGE_SHARE}/scripts/network_launch.py"
[[ -f "$HELPER" ]] || HELPER="${PACKAGE_DIR}/network_launch.py"
RUNTIME_CONFIG="$(python3 "$HELPER" "$CONFIG")"
roslaunch morai_udp_bridge bridge.launch \
    "config:=${RUNTIME_CONFIG}" "${ROSLAUNCH_ARGS[@]}" &
ROSLAUNCH_PID=$!

HEALTH_MONITOR="${PACKAGE_SHARE}/scripts/monitor_packet_health.py"
[[ -f "$HEALTH_MONITOR" ]] || HEALTH_MONITOR="${PACKAGE_DIR}/scripts/monitor_packet_health.py"
python3 "$HEALTH_MONITOR" "$RUNTIME_CONFIG" &
MONITOR_PID=$!

cleanup() {
    kill "$MONITOR_PID" "$ROSLAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
wait "$ROSLAUNCH_PID"
