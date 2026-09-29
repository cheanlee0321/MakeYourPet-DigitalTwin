#!/usr/bin/env python3
"""
Unit tests for Phone A WebSocket command processing, sanitization, and bridge dispatch
"""

import sys
import os
import unittest

SERVER_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SERVER_DIR not in sys.path:
    sys.path.insert(0, SERVER_DIR)

from pilot_server import PilotServerManager

class DummyBridge:
    def __init__(self):
        self.sent_commands = []
        self.connected = True

    def send(self, cmd: str):
        self.sent_commands.append(cmd)
        return True

    def close(self):
        self.connected = False


class TestCommandProcessor(unittest.TestCase):
    def setUp(self):
        web_dir = os.path.join(SERVER_DIR, "pilot_web")
        self.manager = PilotServerManager(
            web_dir=web_dir,
            http_port=0,
            ws_port=0,
            chica_host="127.0.0.1",
            chica_port=18711,
            mock=True  # avoid starting real bridge
        )
        self.dummy_bridge = DummyBridge()
        self.manager.bridge = self.dummy_bridge

    def tearDown(self):
        self.manager.stop()

    def test_walk_onnx_mode(self):
        payload = {
            "type": "walk",
            "mode": "onnx",
            "forward": 0.75,
            "turn": -0.30
        }
        self.manager._process_client_command(payload)
        self.assertEqual(len(self.dummy_bridge.sent_commands), 1)
        self.assertEqual(self.dummy_bridge.sent_commands[0], "walkonnx:-0.300,0.750,0")
        self.assertTrue(self.manager.is_moving)

    def test_walk_tripod_mode(self):
        payload = {
            "type": "walk",
            "mode": "tripod",
            "forward": 0.50,
            "turn": 0.15
        }
        self.manager._process_client_command(payload)
        self.assertEqual(len(self.dummy_bridge.sent_commands), 1)
        self.assertEqual(self.dummy_bridge.sent_commands[0], "walk2:0.150,0.500,0")
        self.assertTrue(self.manager.is_moving)

    def test_walk_boundary_clamping(self):
        payload = {
            "type": "walk",
            "mode": "onnx",
            "forward": 5.0,     # Should clamp to 1.0
            "turn": -8.0        # Should clamp to -1.0
        }
        self.manager._process_client_command(payload)
        self.assertEqual(self.dummy_bridge.sent_commands[0], "walkonnx:-1.000,1.000,0")

        payload_neg = {
            "type": "walk",
            "mode": "onnx",
            "forward": -3.5,    # Should clamp to -1.0
            "turn": 4.0         # Should clamp to 1.0
        }
        self.manager._process_client_command(payload_neg)
        self.assertEqual(self.dummy_bridge.sent_commands[1], "walkonnx:1.000,-1.000,0")

    def test_walk_nan_and_inf_and_none(self):
        payload_nan = {
            "type": "walk",
            "mode": "onnx",
            "forward": float("nan"),
            "turn": float("inf")
        }
        self.manager._process_client_command(payload_nan)
        self.assertEqual(self.dummy_bridge.sent_commands[0], "walkonnx:0.000,0.000,0")

        payload_none = {
            "type": "walk",
            "mode": "onnx",
            "forward": None,
            "turn": None
        }
        self.manager._process_client_command(payload_none)
        self.assertEqual(self.dummy_bridge.sent_commands[1], "walkonnx:0.000,0.000,0")

        payload_str = {
            "type": "walk",
            "mode": "onnx",
            "forward": "invalid",
            "turn": "bad"
        }
        self.manager._process_client_command(payload_str)
        self.assertEqual(self.dummy_bridge.sent_commands[2], "walkonnx:0.000,0.000,0")

    def test_walk_near_zero_deadzone(self):
        payload = {
            "type": "walk",
            "mode": "onnx",
            "forward": 0.01,
            "turn": -0.01
        }
        self.manager._process_client_command(payload)
        self.assertFalse(self.manager.is_moving)

    def test_stop_command(self):
        self.manager.is_moving = True
        self.manager._process_client_command({"type": "stop"})
        self.assertEqual(self.dummy_bridge.sent_commands, ["walkclear"])
        self.assertFalse(self.manager.is_moving)

    def test_estop_command(self):
        self.manager.is_moving = True
        self.manager._process_client_command({"type": "estop"})
        self.assertIn("walkclear", self.dummy_bridge.sent_commands)
        self.assertIn("estop", self.dummy_bridge.sent_commands)
        self.assertFalse(self.manager.is_moving)

    def test_walk_crab_mode(self):
        # In Crab mode: strafe should be mapped to the first parameter (parts[0])
        payload = {
            "type": "walk",
            "mode": "onnx",
            "forward": 0.40,
            "strafe": -0.60,
            "turn": 0.20,
            "crab": True
        }
        self.manager._process_client_command(payload)
        self.assertEqual(self.dummy_bridge.sent_commands[-1], "walkonnx:-0.600,0.400,0")
        self.assertTrue(self.manager.is_moving)

        # Pure lateral strafe without forward/turn should also trigger is_moving
        self.dummy_bridge.sent_commands.clear()
        payload_pure_strafe = {
            "type": "walk",
            "mode": "onnx",
            "forward": 0.0,
            "strafe": 0.50,
            "turn": 0.0,
            "crab": True
        }
        self.manager._process_client_command(payload_pure_strafe)
        self.assertEqual(self.dummy_bridge.sent_commands[-1], "walkonnx:0.500,0.000,0")
        self.assertTrue(self.manager.is_moving)

    def test_raw_commands(self):
        commands = ["torque", "sit", "onnx on", "onnx off", "crab", "clearance on", "calibrate"]
        for cmd in commands:
            self.manager._process_client_command({"type": "cmd", "command": cmd})
        self.assertEqual(self.dummy_bridge.sent_commands, commands)

    def test_command_injection_defense(self):
        # Prevent TCP command splitting by carriage returns / newlines
        self.manager._process_client_command({"type": "cmd", "command": "torque\r\nsit\nreboot"})
        self.assertEqual(self.dummy_bridge.sent_commands, ["torquesitreboot"])

    def test_invalid_packet_types_gracefully(self):
        # Should not throw exception
        self.manager._process_client_command("not a dict")
        self.manager._process_client_command(12345)
        self.manager._process_client_command({})
        self.manager._process_client_command({"type": "unknown_action"})
        self.assertEqual(len(self.dummy_bridge.sent_commands), 0)


if __name__ == "__main__":
    unittest.main()
