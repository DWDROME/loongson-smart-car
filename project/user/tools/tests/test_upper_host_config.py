#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[3]
CONFIG = PROJECT_DIR / "code" / "upper_host_config.h"


class UpperHostConfigTest(unittest.TestCase):
    def test_single_compiled_default_and_key_does_not_override_it(self):
        config = CONFIG.read_text(encoding="utf-8")
        self.assertRegex(config, r'#define UPPER_HOST_IP "10\.156\.129\.81"')
        self.assertRegex(config, r"#define UPPER_HOST_PORT 8086")

        init_h = (PROJECT_DIR / "code" / "init.h").read_text(encoding="utf-8")
        assistant = (PROJECT_DIR / "code" / "AssistantV2.cpp").read_text(encoding="utf-8")
        key_supervisor = (
            PROJECT_DIR / "user" / "tools" / "key_supervisor.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("#define SERVER_IP UPPER_HOST_IP", init_h)
        self.assertIn("constexpr const char *k_default_ip = SERVER_IP;", assistant)
        self.assertNotRegex(key_supervisor, r"ASSISTANT_IP=|--vofa-ip")
        self.assertNotIn("192.168.0.103", key_supervisor)

    def test_remote_reads_the_shared_default(self):
        remote = (
            PROJECT_DIR / "user" / "tools" / "car_remote_tool.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('UPPER_HOST_CONFIG="$SCRIPT_DIR/../../code/upper_host_config.h"', remote)
        self.assertIn('PC_IP="$DEFAULT_PC_IP"', remote)
        self.assertIn('VOFA_PORT="$DEFAULT_VOFA_PORT"', remote)
        self.assertIsNone(re.search(r'PC_IP="\$\{PC_IP:-192\.168\.', remote))
        self.assertNotIn('ASSISTANT_IP=', remote)
        self.assertNotIn('--vofa-ip', remote)

    def test_competition_car_address_and_legacy_switches(self):
        remote = (
            PROJECT_DIR / "user" / "tools" / "car_remote_tool.sh"
        ).read_text(encoding="utf-8")
        build = (PROJECT_DIR / "user" / "build.sh").read_text(encoding="utf-8")
        profile = (
            PROJECT_DIR / "user" / "tools" / "board_network" / "network_profile.conf"
        ).read_text(encoding="utf-8")
        net_switch = (
            PROJECT_DIR / "user" / "tools" / "board_network" / "net-switch"
        ).read_text(encoding="utf-8")

        self.assertIn('SSH_TARGET="root@10.156.129.100"', remote)
        self.assertIn('REMOTE_IP="10.156.129.100"', build)
        self.assertIn('STATIC_IP="10.156.129.100"', profile)
        self.assertIn('GW="10.156.129.181"', profile)
        self.assertIn("competition network is locked", net_switch)
        self.assertNotIn('write_profile "net1"', net_switch)
        self.assertNotIn('write_profile "net0"', net_switch)

    def test_runtime_upper_host_overrides_are_ignored(self):
        main = (PROJECT_DIR / "user" / "main.cpp").read_text(encoding="utf-8")
        assistant = (PROJECT_DIR / "code" / "AssistantV2.cpp").read_text(encoding="utf-8")
        self.assertIn("ignore fixed-upper override", main)
        self.assertNotRegex(main, r'setenv\("ASSISTANT_(IP|PORT)"')
        self.assertIn("g_assistant.ip = k_default_ip;", assistant)
        self.assertIn("g_assistant.port = k_default_port;", assistant)
        self.assertNotIn('read_env_text("ASSISTANT_IP"', assistant)

    def test_net_switch_has_no_mutating_code_path(self):
        net_switch = (
            PROJECT_DIR / "user" / "tools" / "board_network" / "net-switch"
        ).read_text(encoding="utf-8")
        self.assertNotIn("--apply", net_switch)
        self.assertNotRegex(
            net_switch,
            r"write_profile|write_wifi|interactive_setup|rollback\(\)|restart_wifi|ip addr add|ip route replace",
        )


if __name__ == "__main__":
    unittest.main()
