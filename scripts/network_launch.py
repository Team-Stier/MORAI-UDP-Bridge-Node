#!/usr/bin/env python3
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time

import yaml


def log(message):
    print(f"[morai_udp_bridge] {message}", file=sys.stderr, flush=True)


def require_profile(name, profile):
    bridge_ip = str(profile.get("bridge_ip", ""))
    simulator_ip = str(profile.get("simulator_ip", ""))
    if not bridge_ip or not simulator_ip:
        raise RuntimeError(f"{name} profile IP is not configured")
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind((bridge_ip, 0))
        probe.close()
    except OSError as error:
        raise RuntimeError(
            f"{name} bridge_ip is not assigned locally: {bridge_ip}: {error}"
        ) from error
    return bridge_ip, simulator_ip


def receive_probe(name, bridge_ip, simulator_ip, port, timeout):
    log(f"probing {name}: {simulator_ip} -> {bridge_ip}:{port}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((bridge_ip, port))
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            sock.settimeout(remaining)
            try:
                _, sender = sock.recvfrom(65535)
            except socket.timeout:
                return False
            if sender[0] == simulator_ip:
                log(f"received UDP from {sender[0]}:{sender[1]}")
                return True
    finally:
        sock.close()


def rx_ports(config):
    ports = [channel.get("port") for channel in config.get("cameras", {}).values()]
    for name in ("gps", "imu", "status"):
        channel = config.get(name, {})
        if channel.get("enabled", True):
            ports.append(channel.get("port"))
    return sorted({int(port) for port in ports if port})


def open_ufw(config, simulator_ip):
    net = config["network_launch"]
    if not net.get("open_ufw_for_remote", True) or not shutil.which("ufw"):
        return
    status = ""
    for command in (["sudo", "-n", "ufw", "status"], ["ufw", "status"]):
        try:
            status = subprocess.run(
                command, check=True, capture_output=True, text=True
            ).stdout
            break
        except (OSError, subprocess.CalledProcessError):
            continue
    if not status.startswith("Status: active"):
        return
    for port in rx_ports(config):
        try:
            subprocess.run(
                [
                    "sudo", "-n", "ufw", "allow", "from", simulator_ip,
                    "to", "any", "port", str(port), "proto", "udp",
                ],
                check=True,
                stdout=subprocess.DEVNULL,
            )
        except (OSError, subprocess.CalledProcessError) as error:
            raise RuntimeError(
                f"cannot open UDP {port}; grant passwordless ufw permission or open it once manually"
            ) from error


def write_runtime_config(config, bridge_ip, simulator_ip):
    config["bind_ip"] = bridge_ip
    config["allowed_source_ip"] = simulator_ip
    config.setdefault("control", {})["destination_ip"] = simulator_ip
    if config.get("lamp_control", {}).get("enabled", False):
        config["lamp_control"]["destination_ip"] = simulator_ip
    path = os.path.join(tempfile.gettempdir(), f"morai-udp-bridge-{os.getuid()}.yaml")
    with open(path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    os.chmod(path, 0o600)
    return path


def prepare(config_path):
    with open(config_path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    net = config.get("network_launch", {})
    option = int(net.get("launch_option", 1))
    if option not in (1, 2):
        raise RuntimeError("network_launch.launch_option must be 1 or 2")
    port = int(net.get("probe_port", 9303))
    timeout = float(net.get("probe_timeout_sec", 3.0))
    candidates = []
    if option == 1:
        candidates.append(("same-PC", net.get("same_pc", {}), False))
    candidates.append(("remote-PC", net.get("remote_pc", {}), True))

    errors = []
    for name, profile, remote in candidates:
        try:
            bridge_ip, simulator_ip = require_profile(name, profile)
            if remote:
                open_ufw(config, simulator_ip)
            if receive_probe(name, bridge_ip, simulator_ip, port, timeout):
                log(f"selected {name}: bridge={bridge_ip}, simulator={simulator_ip}")
                return write_runtime_config(config, bridge_ip, simulator_ip)
            errors.append(f"{name}: no UDP packet within {timeout:g}s")
        except RuntimeError as error:
            errors.append(str(error))
    raise RuntimeError("; ".join(errors) + "; bridge not launched")


if __name__ == "__main__":
    try:
        if len(sys.argv) != 2:
            raise RuntimeError("usage: network_launch.py CONFIG.yaml")
        print(prepare(sys.argv[1]))
    except (OSError, ValueError, yaml.YAMLError, RuntimeError) as error:
        log(str(error))
        raise SystemExit(2)
