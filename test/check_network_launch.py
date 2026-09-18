#!/usr/bin/env python3
"""Selected profile starts immediately; no UDP probe or automatic fallback."""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
from unittest.mock import patch
import yaml

spec = importlib.util.spec_from_file_location(
    "network_launch", Path(__file__).resolve().parents[1] / "scripts/network_launch.py")
network = importlib.util.module_from_spec(spec)
spec.loader.exec_module(network)


def main():
    route = subprocess.CompletedProcess(
        [], 0, stdout="192.168.0.1 dev enp5s0 src 192.168.0.10\n")
    with patch.object(network.subprocess, "run", return_value=route), \
            patch.object(network.os.path, "exists",
                         side_effect=lambda path: path.endswith("/device")):
        assert network.detect_wired_interface("192.168.0.1") == "enp5s0"

    for option in (1, 2):
        config = {"network_launch": {"launch_option": option, "same_pc": {}, "remote_pc": {}}}
        with tempfile.NamedTemporaryFile(mode="w+", suffix=".yaml") as stream:
            yaml.safe_dump(config, stream)
            stream.flush()
            with patch.object(network.shutil, "which", return_value="/usr/bin/ip"), \
                    patch.object(network, "require_profile", return_value=("127.0.0.1", "127.0.0.1")) as profile, \
                    patch.object(network, "configure_remote_interface") as configure, \
                    patch.object(network, "open_ufw"), \
                    patch.object(network, "write_runtime_config", return_value="runtime.yaml"), \
                    patch.object(network.socket, "socket", side_effect=AssertionError("unexpected probe")):
                assert network.prepare(stream.name) == "runtime.yaml"
                assert profile.call_args.args[0] == ("same-PC" if option == 1 else "remote-PC")
                assert configure.call_count == (option == 2)
    print("PASS: immediate launch, selected profile only")


if __name__ == "__main__":
    main()
