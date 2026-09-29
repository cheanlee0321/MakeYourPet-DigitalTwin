#!/usr/bin/env python3
"""
Integration tests for WebSocket communication, multi-client broadcasting, and TCP bridge resilience
"""

import sys
import os
import time
import json
import socket
import asyncio
import unittest

SERVER_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SERVER_DIR not in sys.path:
    sys.path.insert(0, SERVER_DIR)

from pilot_server import PilotServerManager, ChicaTcpBridge
import websockets

def find_free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class TestNetworkAndWebSocket(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.web_dir = os.path.join(SERVER_DIR, "pilot_web")
        self.ws_port = find_free_port()
        self.manager = PilotServerManager(
            web_dir=self.web_dir,
            http_port=0,
            ws_port=self.ws_port,
            chica_host="127.0.0.1",
            chica_port=18711,
            mock=True,
            start_mock_loop=False  # Do not auto-emit background telemetry
        )
        self.server_task = asyncio.create_task(self.manager.run_ws_server())
        await asyncio.sleep(0.15)  # Allow WS server to start

    async def asyncTearDown(self):
        self.manager.stop()
        self.server_task.cancel()
        try:
            await self.server_task
        except asyncio.CancelledError:
            pass

    async def test_websocket_single_client_telemetry_flow(self):
        url = f"ws://127.0.0.1:{self.ws_port}"
        async with websockets.connect(url) as ws:
            # 1. Send walk command
            await ws.send(json.dumps({
                "type": "walk",
                "mode": "onnx",
                "forward": 0.8,
                "turn": 0.0
            }))

            # 2. Emit telemetry from server
            self.manager._on_telemetry_update({
                "type": "telemetry",
                "robotConnected": True,
                "voltage": 7.42,
                "current": 2.45,
                "bps": 50,
                "legs": "x-x-x-",
                "flags": "110000100"
            })

            # 3. Receive telemetry packet
            raw = await asyncio.wait_for(ws.recv(), timeout=2.0)
            data = json.loads(raw)
            self.assertEqual(data.get("type"), "telemetry")
            self.assertTrue(data.get("robotConnected"))
            self.assertAlmostEqual(data.get("voltage"), 7.42)
            self.assertAlmostEqual(data.get("current"), 2.45)

    async def test_websocket_multi_client_broadcast(self):
        url = f"ws://127.0.0.1:{self.ws_port}"
        async with websockets.connect(url) as ws1, websockets.connect(url) as ws2:
            # Broadcast custom telemetry packet
            test_packet = {
                "type": "telemetry",
                "robotConnected": True,
                "voltage": 7.35,
                "current": 2.80,
                "bps": 50,
                "legs": "x-x-x-",
                "flags": "110000100"
            }
            self.manager._on_telemetry_update(test_packet)

            # Both clients must receive the broadcast
            msg1 = json.loads(await asyncio.wait_for(ws1.recv(), timeout=2.0))
            msg2 = json.loads(await asyncio.wait_for(ws2.recv(), timeout=2.0))

            self.assertAlmostEqual(msg1["voltage"], 7.35)
            self.assertAlmostEqual(msg2["voltage"], 7.35)
            self.assertEqual(msg1["legs"], "x-x-x-")
            self.assertEqual(msg2["legs"], "x-x-x-")


class TestTcpBridgeResilience(unittest.TestCase):
    def test_bridge_auto_reconnect(self):
        """Simulate TCP server drop and verify bridge reconnects cleanly"""
        test_port = find_free_port()
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", test_port))
        listener.listen(1)

        bridge = ChicaTcpBridge(
            host="127.0.0.1",
            port=test_port,
            auto_connect=True
        )

        try:
            # 1. Accept first connection
            conn1, _ = listener.accept()
            with conn1:
                # Wait for bridge to report connected
                for _ in range(20):
                    if bridge.connected:
                        break
                    time.sleep(0.05)
                self.assertTrue(bridge.connected)

                # Send telemetry
                conn1.sendall(b"ready:BPS= 50|V= 7.40|I= 0.60|IP=127.0.0.1|LEGS=xxxxxx|FLAGS=110000100\n")
                time.sleep(0.1)
                self.assertAlmostEqual(bridge.latest_telemetry["voltage"], 7.40)

            # 2. Conn1 is closed now. Temporarily close listener so reconnect attempt fails initially
            listener.close()

            # Wait for bridge to detect disconnect
            for _ in range(30):
                if not bridge.connected:
                    break
                time.sleep(0.05)
            self.assertFalse(bridge.connected)
            self.assertFalse(bridge.latest_telemetry["robotConnected"])

            # 3. Bring listener back online
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", test_port))
            listener.listen(1)

            # 4. Accept second connection after reconnect attempt
            conn2, _ = listener.accept()
            with conn2:
                for _ in range(20):
                    if bridge.connected:
                        break
                    time.sleep(0.05)
                self.assertTrue(bridge.connected)

                conn2.sendall(b"ready:BPS= 50|V= 7.38|I= 1.50|IP=127.0.0.1|LEGS=x-x-x-|FLAGS=110000100\n")
                time.sleep(0.1)
                self.assertAlmostEqual(bridge.latest_telemetry["voltage"], 7.38)
                self.assertTrue(bridge.latest_telemetry["robotConnected"])

        finally:
            bridge.close()
            try:
                listener.close()
            except Exception:
                pass


if __name__ == "__main__":
    unittest.main()
