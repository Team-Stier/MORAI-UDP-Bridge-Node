#!/usr/bin/env bash
set -e
exec roslaunch morai_udp_bridge bridge.launch "$@"
