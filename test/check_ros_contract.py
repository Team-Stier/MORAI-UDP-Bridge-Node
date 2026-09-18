#!/usr/bin/env python3
"""Run with the workspace sourced. Uses a private master and loopback UDP only."""
import copy
import os
from pathlib import Path
import socket
import signal
import struct
import subprocess
import time
import tempfile

import yaml


def free_port(kind=socket.SOCK_DGRAM):
    with socket.socket(socket.AF_INET, kind) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_for(predicate, description, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError(description)


def stop(process):
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    port = free_port(socket.SOCK_STREAM)
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
    os.environ["ROS_IP"] = "127.0.0.1"
    os.environ.pop("ROS_HOSTNAME", None)
    master = subprocess.Popen(["roscore", "-p", str(port)], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    node = None
    try:
        import rosgraph
        import rospy
        from diagnostic_msgs.msg import DiagnosticArray
        from morai_msgs.msg import CtrlCmd
        from sensor_msgs.msg import CameraInfo, CompressedImage

        wait_for(rosgraph.is_master_online, "private master did not start")
        rospy.init_node("contract_check", disable_signals=True)
        config = yaml.safe_load((Path(__file__).resolve().parents[1] / "config/config.yaml").read_text())
        config.update(bind_ip="127.0.0.1", allowed_source_ip="127.0.0.1")
        config["vehicle_spec"] = dict(base_frame_id="base_link", gps_frame_id="gps", imu_frame_id="imu",
            cameras={name: dict(frame_id=name, width=640, height=480, horizontal_fov_deg=90.0)
                     for name in config["cameras"]})
        for channel in list(config["cameras"].values()) + [config[name] for name in ("gps", "imu", "status")]:
            channel["port"] = free_port()
        config["control"].update(destination_ip="127.0.0.1", destination_port=free_port())
        latest, high, images, infos = {}, [], [], []

        def collect_latest(message):
            high.append(message)
            for status in message.status:
                latest[status.name.rsplit("/", 1)[-1]] = dict((v.key, v.value) for v in status.values)

        subscriptions = [
            rospy.Subscriber("/diagnostics/high_hz", DiagnosticArray, collect_latest, queue_size=100),
            rospy.Subscriber(config["cameras"]["front"]["topic"], CompressedImage, images.append, queue_size=10),
            rospy.Subscriber(config["cameras"]["front"]["camera_info_topic"], CameraInfo, infos.append, queue_size=10),
        ]
        command = rospy.Publisher(config["control"]["command_topic"], CtrlCmd, queue_size=1)

        node_log = tempfile.TemporaryFile(mode="w+")

        def launch(settings):
            rospy.set_param("/contract_bridge", settings)
            return subprocess.Popen(["rosrun", "morai_udp_bridge", "morai_udp_bridge_node", "__name:=contract_bridge"],
                                    stdout=node_log, stderr=subprocess.STDOUT)

        node = launch(config)
        wait_for(lambda: command.get_num_connections() and "control_tx" in latest and
                 "camera_camera_7" in latest, "channels not initialized")
        wait_for(lambda: latest["control_tx"].get("sent", "0") != "0", "TX not sending")
        assert latest["control_tx"]["error_code"] == "STALE_COMMAND", latest["control_tx"]
        assert all(key in latest["control_tx"] for key in ("state_since", "last_error_time", "last_transition_time", "command_age_ms"))
        assert "hz" not in latest["control_tx"] and "lifetime_p95_ms" not in latest["control_tx"]
        assert "/diagnostics" not in dict(rospy.get_published_topics())
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            def nmea(body):
                checksum = 0
                for char in body:
                    checksum ^= ord(char)
                return ("$%s*%02X\r\n" % (body, checksum)).encode()

            node.send_signal(signal.SIGSTOP)
            time.sleep(0.05)
            gps_address = ("127.0.0.1", config["gps"]["port"])
            udp.sendto(nmea("GPGGA,123519,3723.2475,N,12702.3416,E,1,03,5.2,10.0,M,20.0,M,,"), gps_address)
            udp.sendto(nmea("GPRMC,123519"), gps_address)
            udp.sendto(nmea("GPGGA,broken"), gps_address)
            udp.sendto(nmea("GPGGA,broken"), gps_address)
            node.send_signal(signal.SIGCONT)
            wait_for(lambda: latest.get("gps", {}).get("published") == "1", "GPS fallback not published")
            assert latest["gps"]["parse_error"] == "2", latest["gps"]
            assert latest["gps"]["non_gga"] == "1", latest["gps"]
            assert any(s.name.endswith("/gps") and s.level == 1 and
                       any(v.key == "error_code" and v.value == "STRUCTURAL_ERROR" for v in s.values)
                       for msg in high for s in msg.status)
            destination = ("127.0.0.1", config["cameras"]["front"]["port"])

            def chunk(index, payload, final=False, frame=1):
                udp.sendto(b"MOR" + struct.pack("<IIII", frame, 0, index, len(payload)) + payload +
                           (b"EI" if final else b"AI"), destination)

            first_before = rospy.Time.now().to_sec()
            chunk(0, b"\xff\xd8\x01")
            time.sleep(0.08)
            last_before = rospy.Time.now().to_sec()
            chunk(1, b"\x02\xff\xd9", True)
            wait_for(lambda: images and infos, "camera frame not published")
            assert first_before <= images[-1].header.stamp.to_sec() < last_before
            assert images[-1].header == infos[-1].header
            chunk(0, b"broken", True, 2)
            chunk(0, b"\xff\xd8", frame=3)
            chunk(0, b"\xff\xd8\xff\xd9", True, 4)
            wait_for(lambda: int(latest["camera_front"].get("completed_frames", 0)) == 2, "camera counters not updated")
            camera = latest["camera_front"]
            assert int(camera["chunks"]) == 5 and int(camera["invalid_frames"]) == 1, camera
            assert int(camera["dropped_frames"]) >= 2, camera
            node.send_signal(signal.SIGSTOP)
            time.sleep(0.05)
            for frame in range(100, 140):
                chunk(0, b"\xff\xd8\xff\xd9", True, frame)
            node.send_signal(signal.SIGCONT)
            wait_for(lambda: int(latest["camera_front"].get("completed_frames", 0)) == 42,
                     "bounded camera batches lost chunks")
        command.publish(CtrlCmd(accel=float("nan"), brake=1.4, steering=-2.0))
        wait_for(lambda: latest["control_tx"].get("sanitized") == "1", "sanitize not recorded")
        assert "nan" in latest["control_tx"]["before"], latest["control_tx"]
        assert "accel=0,brake=1,steering=-1" == latest["control_tx"]["after"]
        assert any(any(v.key == "before" and "nan" in v.value for s in msg.status for v in s.values) for msg in high)
        stop(node)

        broken = copy.deepcopy(config)
        for name in ("gps", "imu", "status"):
            broken[name]["port"] = -1
        broken["control"]["destination_port"] = -1
        broken["lamp_control"].update(enabled=True, destination_port=-1)
        broken["cameras"]["left"]["camera_info_topic"] = "invalid topic"
        del broken["vehicle_spec"]["cameras"]["right"]["width"]
        latest.clear()
        node = launch(broken)
        failed = ("gps", "imu", "status", "control_tx", "lamp_tx", "camera_left", "camera_right")
        def isolated_errors_logged():
            node_log.seek(0)
            output = node_log.read()
            return all(name + ": CONFIG_ERROR" in output for name in failed)

        wait_for(isolated_errors_logged, "configuration errors escaped channel boundary")
        assert node.poll() is None
        print("PASS: first-chunk stamp, diagnostics/counters, sanitize, channel isolation")
    finally:
        stop(node)
        stop(master)


if __name__ == "__main__":
    main()
