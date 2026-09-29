#!/usr/bin/env python3
"""
Unit tests for Chica TCP status line parsing & telemetry extraction
"""

import sys
import os
import unittest

# Ensure pilot_server can be imported
SERVER_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SERVER_DIR not in sys.path:
    sys.path.insert(0, SERVER_DIR)

from pilot_server import ChicaTcpBridge

class TestProtocolParser(unittest.TestCase):
    def setUp(self):
        self.received_updates = []
        self.bridge = ChicaTcpBridge(
            host="127.0.0.1",
            port=18711,
            on_telemetry=lambda t: self.received_updates.append(t),
            auto_connect=False
        )

    def tearDown(self):
        self.bridge.close()

    def test_parse_standard_ready_line(self):
        sample = "ready:BPS= 50|V= 7.420|I= 0.650|IP=127.0.0.1|LEGS=xxxxxx|FLAGS=110000100"
        self.bridge._parse_line(sample)

        telem = self.bridge.latest_telemetry
        self.assertTrue(telem["robotConnected"])
        self.assertEqual(telem["bps"], 50)
        self.assertAlmostEqual(telem["voltage"], 7.420, places=3)
        self.assertAlmostEqual(telem["current"], 0.650, places=3)
        self.assertEqual(telem["ip"], "127.0.0.1")
        self.assertEqual(telem["legs"], "xxxxxx")
        self.assertEqual(telem["flags"], "110000100")
        self.assertEqual(len(self.received_updates), 1)

    def test_parse_busy_line_with_dashes(self):
        sample = "busy:BPS= 0|V=---  |I=---  |IP=192.168.4.1|LEGS=------|FLAGS=000000000"
        self.bridge._parse_line(sample)

        telem = self.bridge.latest_telemetry
        self.assertTrue(telem["robotConnected"])
        self.assertEqual(telem["bps"], 0)
        self.assertIsNone(telem["voltage"])
        self.assertIsNone(telem["current"])
        self.assertEqual(telem["ip"], "192.168.4.1")
        self.assertEqual(telem["legs"], "------")
        self.assertEqual(telem["flags"], "000000000")

    def test_parse_corrupted_numeric_fields(self):
        sample = "ready:BPS=not_an_int|V=invalid_volt|I=corrupt_amp|IP=10.0.0.2|LEGS=x-x-x-|FLAGS=101010101"
        self.bridge._parse_line(sample)

        telem = self.bridge.latest_telemetry
        self.assertTrue(telem["robotConnected"])
        # bps, voltage, current should keep defaults without throwing exceptions
        self.assertEqual(telem["bps"], 0)
        self.assertIsNone(telem["voltage"])
        self.assertIsNone(telem["current"])
        self.assertEqual(telem["legs"], "x-x-x-")
        self.assertEqual(telem["flags"], "101010101")

    def test_parse_partial_and_reordered_fields(self):
        sample = "ready:FLAGS=100000000|LEGS=-x-x-x|V= 6.85"
        self.bridge._parse_line(sample)

        telem = self.bridge.latest_telemetry
        self.assertEqual(telem["flags"], "100000000")
        self.assertEqual(telem["legs"], "-x-x-x")
        self.assertAlmostEqual(telem["voltage"], 6.85, places=2)

    def test_parse_malformed_lines_gracefully(self):
        # Empty string, whitespace, non-conforming lines should not raise
        self.bridge._parse_line("")
        self.bridge._parse_line("   \t  \n")
        self.bridge._parse_line("unrecognized_command_output")
        self.bridge._parse_line("error:something broke")
        self.assertEqual(len(self.received_updates), 0)

    def test_tcp_framing_buffer_simulation(self):
        """Simulate chunked TCP socket reception with multi-line splitting"""
        chunks = [
            "ready:BPS=50|V=7.4|",
            "I=1.2|LEGS=xxxxxx|FLAGS=11\nready:BPS=48|V=",
            "7.3|I=1.1|LEGS=------|FLAGS=00\n"
        ]

        parsed_lines = []
        buf = ""
        for chunk in chunks:
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line:
                    self.bridge._parse_line(line)
                    parsed_lines.append(line)

        self.assertEqual(len(parsed_lines), 2)
        self.assertEqual(len(self.received_updates), 2)
        self.assertEqual(self.bridge.latest_telemetry["bps"], 48)
        self.assertAlmostEqual(self.bridge.latest_telemetry["voltage"], 7.3, places=1)
        self.assertEqual(self.bridge.latest_telemetry["legs"], "------")


if __name__ == "__main__":
    unittest.main()
