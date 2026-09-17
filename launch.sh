#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${MORAI_BRIDGE_CONFIG:-${PACKAGE_DIR}/config/config.yaml}"
ROSLAUNCH_ARGS=()

for arg in "$@"; do
    case "$arg" in
        config:=*) CONFIG="${arg#config:=}" ;;
        *) ROSLAUNCH_ARGS+=("$arg") ;;
    esac
done

HELPER="${PACKAGE_DIR}/scripts/network_launch.py"
[[ -f "$HELPER" ]] || HELPER="${PACKAGE_DIR}/network_launch.py"
RUNTIME_CONFIG="$(python3 "$HELPER" "$CONFIG")"
exec roslaunch morai_udp_bridge bridge.launch \
    "config:=${RUNTIME_CONFIG}" "${ROSLAUNCH_ARGS[@]}"
