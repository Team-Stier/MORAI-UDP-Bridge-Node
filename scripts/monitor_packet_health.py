#!/usr/bin/env python3
import sys

import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticArray


def enabled_channels(config):
    channels = [f"camera_{name}" for name in config.get("cameras", {})]
    channels += [name for name in ("gps", "imu", "status")
                 if config.get(name, {}).get("enabled", True)]
    channels.append("control_tx")
    if config.get("lamp_control", {}).get("enabled", False):
        channels.append("lamp_tx")
    return set(channels)


class HealthMonitor:
    def __init__(self, expected):
        self.expected = expected
        self.states = {}

    def callback(self, message):
        for status in message.status:
            channel = status.name.rsplit("/", 1)[-1]
            if channel not in self.expected:
                continue
            values = {item.key: item.value for item in status.values}
            healthy = status.level == 0 and values.get("error_code") in {
                "PUBLISHED", "SENT"
            }
            if self.states.get(channel) == healthy:
                continue
            self.states[channel] = healthy
            state = "GREEN" if healthy else "RED"
            print(
                f"[morai_udp_bridge] {state} {channel}: "
                f"{status.message} received={values.get('received', '0')} "
                f"published={values.get('published', '0')} "
                f"sent={values.get('sent', '0')} "
                f"parse_error={values.get('parse_error', '0')} "
                f"send_error={values.get('send_error', '0')}",
                flush=True,
            )


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: monitor_packet_health.py CONFIG.yaml")
    with open(sys.argv[1], encoding="utf-8") as stream:
        expected = enabled_channels(yaml.safe_load(stream) or {})
    print(f"[morai_udp_bridge] packet health monitor: {', '.join(sorted(expected))}",
          flush=True)
    rospy.init_node("morai_udp_bridge_packet_health", anonymous=True)
    monitor = HealthMonitor(expected)
    rospy.Subscriber("/diagnostics", DiagnosticArray, monitor.callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
