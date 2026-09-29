#!/usr/bin/env python3
"""
Unit and timing tests for Deadman Switch safety watchdog
"""

import sys
import os
import time
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


class TestDeadmanWatchdog(unittest.TestCase):
    def setUp(self):
        web_dir = os.path.join(SERVER_DIR, "pilot_web")
        # Use a short timeout (80ms) for fast and deterministic unit testing
        self.manager = PilotServerManager(
            web_dir=web_dir,
            http_port=0,
            ws_port=0,
            chica_host="127.0.0.1",
            chica_port=18711,
            mock=True,
            deadman_timeout=0.08
        )
        self.dummy_bridge = DummyBridge()
        self.manager.bridge = self.dummy_bridge

    def tearDown(self):
        self.manager.stop()

    def test_deadman_triggers_when_signal_lost(self):
        # 1. Start moving
        self.manager._process_client_command({"type": "walk", "mode": "onnx", "forward": 0.5})
        self.assertTrue(self.manager.is_moving)
        self.dummy_bridge.sent_commands.clear()

        # 2. Wait for watchdog timeout (> 80ms)
        time.sleep(0.12)

        # 3. Verify auto-braking
        self.assertIn("walkclear", self.dummy_bridge.sent_commands)
        self.assertFalse(self.manager.is_moving)

    def test_deadman_does_not_trigger_when_stationary(self):
        # Ensure robot is stationary
        self.manager.is_moving = False
        self.dummy_bridge.sent_commands.clear()

        # Wait longer than timeout
        time.sleep(0.12)

        # No spurious commands should be sent
        self.assertEqual(len(self.dummy_bridge.sent_commands), 0)
        self.assertFalse(self.manager.is_moving)

    def test_deadman_refreshed_by_active_stream(self):
        # Stream joystick packets every 25ms for 150ms (total duration > timeout of 80ms)
        start_t = time.time()
        while time.time() - start_t < 0.15:
            self.manager._process_client_command({"type": "walk", "mode": "onnx", "forward": 0.4})
            time.sleep(0.025)

        # Verify walkclear was never sent while signal was actively streaming
        self.assertNotIn("walkclear", self.dummy_bridge.sent_commands)
        self.assertTrue(self.manager.is_moving)

        # Now stop sending and wait for timeout
        time.sleep(0.12)

        # Now walkclear should have triggered
        self.assertIn("walkclear", self.dummy_bridge.sent_commands)
        self.assertFalse(self.manager.is_moving)


if __name__ == "__main__":
    unittest.main()
