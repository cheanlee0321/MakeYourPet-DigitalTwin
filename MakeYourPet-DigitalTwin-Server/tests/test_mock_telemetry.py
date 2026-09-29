#!/usr/bin/env python3
"""
Unit tests for mock telemetry generator (stationarity, tripod alternating, voltage/current ranges)
"""

import sys
import os
import time
import unittest

SERVER_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SERVER_DIR not in sys.path:
    sys.path.insert(0, SERVER_DIR)

from pilot_server import PilotServerManager

class TestMockTelemetry(unittest.TestCase):
    def setUp(self):
        web_dir = os.path.join(SERVER_DIR, "pilot_web")
        self.received_telemetry = []
        self.manager = PilotServerManager(
            web_dir=web_dir,
            http_port=0,
            ws_port=0,
            chica_host="127.0.0.1",
            chica_port=18711,
            mock=True,
            deadman_timeout=1.0  # Allow longer test window
        )
        self.manager._on_telemetry_update = lambda t: self.received_telemetry.append(t)

    def tearDown(self):
        self.manager.stop()

    def test_mock_stationary_telemetry(self):
        self.manager.is_moving = False
        time.sleep(0.12)  # Capture a few 25Hz frames
        self.assertGreater(len(self.received_telemetry), 0)

        for telem in self.received_telemetry[-3:]:
            self.assertTrue(telem["robotConnected"])
            self.assertEqual(telem["legs"], "xxxxxx", "Stationary pose must report all 6 feet on ground")
            self.assertAlmostEqual(telem["current"], 0.58, places=1)
            self.assertGreater(telem["voltage"], 7.0)
            self.assertLess(telem["voltage"], 8.0)
            self.assertEqual(telem["bps"], 50)

    def test_mock_moving_telemetry(self):
        self.received_telemetry.clear()

        # Actively stream walk commands at 25Hz for 0.45s
        start_t = time.time()
        while time.time() - start_t < 0.45:
            self.manager._process_client_command({"type": "walk", "mode": "onnx", "forward": 0.5})
            time.sleep(0.04)

        self.assertGreater(len(self.received_telemetry), 5)
        observed_legs = set(t["legs"] for t in self.received_telemetry)

        # In motion, tripod phases alternate between 'x-x-x-' and '-x-x-x'
        self.assertTrue("x-x-x-" in observed_legs or "-x-x-x" in observed_legs)

        for t in self.received_telemetry:
            # Walking current should be elevated (> 1.8A)
            self.assertGreater(t["current"], 1.8)
            self.assertLess(t["current"], 3.0)


if __name__ == "__main__":
    unittest.main()
