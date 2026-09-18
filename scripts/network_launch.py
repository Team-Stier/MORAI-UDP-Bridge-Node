#!/usr/bin/env python3
import os
import shutil
import socket
import subprocess
import sys
import tempfile

import yaml


def log(message):
    print(f"[morai_udp_bridge] {message}", file=sys.stderr, flush=True)


def run_privileged(command):
    prefix = [] if os.geteuid() == 0 else ["sudo"]
    subprocess.run(prefix + command, check=True)


def assigned_interface(ip):
    result = subprocess.run(
        ["ip", "-o", "-4", "addr", "show"],
        check=True, capture_output=True, text=True,
    )
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) >= 4 and fields[3].split("/", 1)[0] == ip:
            return fields[1]
    return ""


def detect_wired_interface(target_ip):
    route = subprocess.run(
        ["ip", "-o", "route", "get", target_ip],
        capture_output=True, text=True,
    )
    fields = route.stdout.split()
    if route.returncode == 0 and "dev" in fields:
        index = fields.index("dev") + 1
        if index < len(fields):
            name = fields[index]
            root = f"/sys/class/net/{name}"
            if os.path.exists(f"{root}/device") and not os.path.exists(f"{root}/wireless"):
                return name

    candidates = []
    for name in sorted(os.listdir("/sys/class/net")):
        root = f"/sys/class/net/{name}"
        if name == "lo" or os.path.exists(f"{root}/wireless") or not os.path.exists(f"{root}/device"):
            continue
        try:
            with open(f"{root}/carrier", encoding="ascii") as stream:
                if stream.read().strip() == "1":
                    candidates.append(name)
        except OSError:
            continue
    if len(candidates) != 1:
        raise RuntimeError(
            "cannot uniquely detect connected wired NIC: " +
            (", ".join(candidates) if candidates else "none")
        )
    return candidates[0]


def configure_remote_interface(profile):
    if not shutil.which("ip"):
        log("ip command unavailable; using preconfigured host network")
        return
    bridge_ip = str(profile.get("bridge_ip", ""))
    simulator_ip = str(profile.get("simulator_ip", ""))
    interface = assigned_interface(bridge_ip)
    if not interface:
        interface = str(profile.get("interface", "")) or detect_wired_interface(simulator_ip)
        prefix = int(profile.get("prefix_length", 24))
        log(f"configuring wired NIC {interface}: {bridge_ip}/{prefix}")
        run_privileged(["ip", "link", "set", "dev", interface, "up"])
        run_privileged(["ip", "addr", "replace", f"{bridge_ip}/{prefix}", "dev", interface])
    run_privileged([
        "ip", "route", "replace", f"{simulator_ip}/32", "dev", interface,
        "src", bridge_ip,
    ])


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


def write_runtime_config(config, bridge_ip, simulator_ip, allowed_source_ip=None):
    config["bind_ip"] = bridge_ip
    config["allowed_source_ip"] = simulator_ip if allowed_source_ip is None else allowed_source_ip
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
    name = "same-PC" if option == 1 else "remote-PC"
    profile = net.get("same_pc" if option == 1 else "remote_pc", {})
    remote = option == 2

    if remote and not shutil.which("ip"):
        bridge_ip = "0.0.0.0"
        simulator_ip = str(profile.get("simulator_ip", ""))
        if not simulator_ip:
            raise RuntimeError(f"{name} simulator_ip is not configured")
        open_ufw(config, simulator_ip)
        log(f"selected {name}: bridge={bridge_ip}, simulator={simulator_ip}")
        return write_runtime_config(config, bridge_ip, simulator_ip, "")

    if remote:
        configure_remote_interface(profile)
    bridge_ip, simulator_ip = require_profile(name, profile)
    if remote:
        open_ufw(config, simulator_ip)
    log(f"selected {name}: bridge={bridge_ip}, simulator={simulator_ip}")
    return write_runtime_config(config, bridge_ip, simulator_ip)


if __name__ == "__main__":
    try:
        if len(sys.argv) != 2:
            raise RuntimeError("usage: network_launch.py CONFIG.yaml")
        print(prepare(sys.argv[1]))
    except (OSError, ValueError, yaml.YAMLError, RuntimeError) as error:
        log(str(error))
        raise SystemExit(2)
