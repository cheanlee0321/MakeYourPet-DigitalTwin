#!/usr/bin/env python3
"""
MakeYourPet Pilot Server - Phone A Teleoperation Relay & Web Server
- Serves HTML5/Canvas tactical HUD for Phone A browser (Port 8080)
- Establishes ultra-low-latency bidirectional WebSocket channel (Port 8081)
- Bridges to Phone B (chica-server TCP 18711), supporting both ONNX policy and original gait
- Built-in Deadman Switch double-timeout watchdog and telemetry parser
"""

import os
import sys
import math

# Ensure UTF-8 output under Windows cp950 environment
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")

import time
import json
import socket
import select
import asyncio
import argparse
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

try:
    import websockets
    from websockets.asyncio.server import serve as ws_serve
except ImportError:
    print("[Error] Please make sure websockets is installed: pip install websockets")
    sys.exit(1)

# --- Default Parameters ---
DEFAULT_HTTP_PORT = 8080
DEFAULT_WS_PORT = 8081
DEFAULT_CHICA_HOST = "127.0.0.1"  # Local simulation or hotspot IP (e.g. 192.168.43.1)
DEFAULT_CHICA_PORT = 18711
DEADMAN_TIMEOUT_SEC = 0.35        # Auto-brake if no command received for 350ms

class ChicaTcpBridge:
    """Manages persistent connection and bidirectional data transfer with Phone B (ChicaServer TCP 18711)"""
    def __init__(self, host: str, port: int, on_telemetry=None, auto_connect: bool = True):
        self.host = host
        self.port = port
        self.on_telemetry = on_telemetry
        self.sock = None
        self.connected = False
        self.running = True
        self.lock = threading.Lock()
        self.latest_telemetry = {
            "type": "telemetry",
            "robotConnected": False,
            "ip": host,
            "voltage": None,
            "current": None,
            "bps": 0,
            "legs": "------",
            "flags": "000000000"
        }
        self.thread = None
        if auto_connect:
            self.thread = threading.Thread(target=self._connection_loop, daemon=True)
            self.thread.start()

    def _connection_loop(self):
        while self.running:
            if not self.connected:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(1.0)
                    s.connect((self.host, self.port))
                    with self.lock:
                        self.sock = s
                        self.connected = True
                        self.latest_telemetry["robotConnected"] = True
                        current_telem = dict(self.latest_telemetry)
                    print(f"[*] Successfully connected to ChicaServer [{self.host}:{self.port}]")
                    if self.on_telemetry:
                        self.on_telemetry(current_telem)
                except Exception as e:
                    with self.lock:
                        self.sock = None
                        self.connected = False
                        self.latest_telemetry["robotConnected"] = False
                    time.sleep(1.0)
                    continue

            # Telemetry receive loop
            buf = ""
            while self.running and self.connected:
                with self.lock:
                    s = self.sock
                if not s:
                    break
                try:
                    r, _, _ = select.select([s], [], [], 0.5)
                    if not r:
                        continue
                    chunk = s.recv(2048).decode('utf-8', errors='ignore')
                    if not chunk:
                        if self.running:
                            print("[-] ChicaServer connection closed")
                        break
                    buf += chunk
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            self._parse_line(line)
                except Exception as e:
                    if self.running:
                        print(f"[-] Read error: {e}")
                    break

            with self.lock:
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                    self.sock = None
                self.connected = False
                self.latest_telemetry["robotConnected"] = False
                offline_telem = dict(self.latest_telemetry)
            if self.on_telemetry:
                self.on_telemetry(offline_telem)
            time.sleep(1.0)

    def _parse_line(self, line: str):
        """Parse Chica server status line: ready:BPS=...|V=...|I=...|IP=...|LEGS=...|FLAGS=..."""
        if line.startswith("ready:") or line.startswith("busy:"):
            payload = line.split(":", 1)[1]
            parts = payload.split("|")
            with self.lock:
                telem = self.latest_telemetry.copy()
            telem["robotConnected"] = True

            for p in parts:
                if p.startswith("BPS="):
                    try:
                        telem["bps"] = int(p[4:].strip())
                    except ValueError:
                        pass
                elif p.startswith("V="):
                    try:
                        val = p[2:].strip()
                        telem["voltage"] = float(val) if val != "---" else None
                    except ValueError:
                        pass
                elif p.startswith("I="):
                    try:
                        val = p[2:].strip()
                        telem["current"] = float(val) if val != "---" else None
                    except ValueError:
                        pass
                elif p.startswith("IP="):
                    telem["ip"] = p[3:].strip()
                elif p.startswith("LEGS="):
                    telem["legs"] = p[5:].strip()
                elif p.startswith("FLAGS="):
                    telem["flags"] = p[6:].strip()

            with self.lock:
                self.latest_telemetry = telem
            if self.on_telemetry:
                try:
                    self.on_telemetry(telem)
                except Exception as e:
                    print(f"[-] Telemetry callback error: {e}")

    def send(self, cmd: str):
        """Send text command to ChicaServer (automatically appends \\n)"""
        with self.lock:
            if not self.connected or not self.sock:
                return False
            try:
                line = (cmd.strip() + "\n").encode('utf-8')
                self.sock.sendall(line)
                return True
            except Exception as e:
                print(f"[-] Failed to send command [{cmd}]: {e}")
                self.connected = False
                return False

    def close(self):
        self.running = False
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None

class PilotServerManager:
    """Coordinates HTTP server, WebSocket service, and TCP bridge"""
    def __init__(self, web_dir: str, http_port: int, ws_port: int, chica_host: str, chica_port: int, mock: bool = False, deadman_timeout: float = DEADMAN_TIMEOUT_SEC, start_mock_loop: bool = True):
        self.web_dir = web_dir
        self.http_port = http_port
        self.ws_port = ws_port
        self.chica_host = chica_host
        self.chica_port = chica_port
        self.mock = mock
        self.deadman_timeout = deadman_timeout

        self.running = True
        self.httpd = None
        self.http_thread = None
        self.ws_server = None
        self.ws_clients = set()
        self.ws_loop = None
        self.last_move_time = time.time()
        self.is_moving = False

        if not mock:
            self.bridge = ChicaTcpBridge(chica_host, chica_port, on_telemetry=self._on_telemetry_update)
        else:
            self.bridge = None
            if start_mock_loop:
                threading.Thread(target=self._mock_telemetry_loop, daemon=True).start()

        # Start Deadman watchdog thread
        threading.Thread(target=self._deadman_watchdog, daemon=True).start()

    def _on_telemetry_update(self, telemetry_dict):
        """Broadcast received robot telemetry to all Phone A clients via WebSocket"""
        if not self.ws_loop or not self.running:
            return
        payload = json.dumps(telemetry_dict)
        try:
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self.ws_loop)
        except RuntimeError:
            pass

    async def _broadcast(self, message: str):
        if not self.ws_clients:
            return
        to_remove = set()
        for client in list(self.ws_clients):
            try:
                await client.send(message)
            except Exception:
                to_remove.add(client)
        self.ws_clients.difference_update(to_remove)

    def _deadman_watchdog(self):
        """Automatically send walkclear if no control command received within deadman_timeout and currently moving"""
        while self.running:
            time.sleep(0.02)
            if self.is_moving and (time.time() - self.last_move_time > self.deadman_timeout):
                print("[Warning - Deadman Watchdog] Joystick signal timed out, auto-braking (walkclear)")
                self.is_moving = False
                if self.bridge:
                    self.bridge.send("walkclear")

    def _mock_telemetry_loop(self):
        """Mock mode: generates virtual voltage, current, and foot contact signals for offline frontend testing"""
        print("[*] Mock mode enabled, generating simulated Chica telemetry")
        step = 0
        while self.running:
            time.sleep(0.04) # 25Hz
            step += 1
            # In stationary stance, all 6 feet firmly touch ground (xxxxxx); alternate tripod steps only when moving
            if self.is_moving:
                legs = "x-x-x-" if (step % 20 < 10) else "-x-x-x"
                i = 2.25 + 0.15 * math.sin(step * 0.3)
                v = 7.36 + 0.02 * math.cos(step * 0.1)
            else:
                legs = "xxxxxx"  # All 6 feet on ground without flickering
                i = 0.58
                v = 7.42

            mock_data = {
                "type": "telemetry",
                "robotConnected": True,
                "ip": "127.0.0.1 (MOCK)",
                "voltage": round(v, 2),
                "current": round(i, 2),
                "bps": 50,
                "legs": legs,
                "flags": "110000100"
            }
            self._on_telemetry_update(mock_data)

    async def handle_ws_client(self, websocket):
        """Handle single Phone A WebSocket client connection"""
        self.ws_clients.add(websocket)
        print(f"[+] Phone A client connected: {websocket.remote_address}")

        # Send latest state immediately
        if self.bridge:
            with self.bridge.lock:
                telem_to_send = dict(self.bridge.latest_telemetry)
            await websocket.send(json.dumps(telem_to_send))

        try:
            async for raw in websocket:
                try:
                    data = json.loads(raw)
                    self._process_client_command(data)
                except Exception as e:
                    print(f"[-] Client packet handling error: {e}")
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(websocket)
            print(f"[-] Phone A client disconnected: {websocket.remote_address}")

    def _process_client_command(self, data: dict):
        """Parse control command from Phone A and forward to ChicaServer"""
        if not isinstance(data, dict):
            return

        cmd_type = data.get("type")

        if cmd_type == "walk":
            mode = str(data.get("mode", "onnx")).lower()
            try:
                forward_val = data.get("forward", 0.0)
                forward = float(forward_val) if forward_val is not None else 0.0
                if math.isnan(forward) or math.isinf(forward):
                    forward = 0.0
            except (ValueError, TypeError):
                forward = 0.0

            try:
                turn_val = data.get("turn", 0.0)
                turn = float(turn_val) if turn_val is not None else 0.0
                if math.isnan(turn) or math.isinf(turn):
                    turn = 0.0
            except (ValueError, TypeError):
                turn = 0.0

            try:
                strafe_val = data.get("strafe", 0.0)
                strafe = float(strafe_val) if strafe_val is not None else 0.0
                if math.isnan(strafe) or math.isinf(strafe):
                    strafe = 0.0
            except (ValueError, TypeError):
                strafe = 0.0

            is_crab = bool(data.get("crab", False))

            # Clamp forward, turn, and strafe to [-1.0, 1.0] safe bounds
            forward = max(-1.0, min(1.0, forward))
            turn = max(-1.0, min(1.0, turn))
            strafe = max(-1.0, min(1.0, strafe))

            # In Crab mode, Chica protocol expects strafe in parts[0]
            horiz = strafe if is_crab else turn

            # Determine command string based on mode
            # ONNX gait: walkonnx:horiz,forward,0
            # Classical tripod gait: walk2:horiz,forward,0
            prefix = "walkonnx:" if mode == "onnx" else "walk2:"
            cmd = f"{prefix}{horiz:.3f},{forward:.3f},0"

            self.is_moving = (abs(forward) > 0.02 or abs(turn) > 0.02 or (is_crab and abs(strafe) > 0.02))
            self.last_move_time = time.time()

            if self.bridge:
                self.bridge.send(cmd)

        elif cmd_type == "stop":
            self.is_moving = False
            self.last_move_time = time.time()
            if self.bridge:
                self.bridge.send("walkclear")

        elif cmd_type == "estop":
            self.is_moving = False
            self.last_move_time = time.time()
            if self.bridge:
                self.bridge.send("walkclear")
                self.bridge.send("estop")

        elif cmd_type == "cmd":
            raw_cmd = str(data.get("command", "") or "")
            cleaned_cmd = raw_cmd.strip().replace("\r", "").replace("\n", "")
            if cleaned_cmd and self.bridge:
                self.bridge.send(cleaned_cmd)

    def start_http_server(self):
        """Start static web server (Thread)"""
        web_dir = self.web_dir

        class StaticHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=web_dir, **kwargs)

            def log_message(self, format, *args):
                pass  # Silence HTTP request logs to keep terminal output clean

        self.httpd = ThreadingHTTPServer(("0.0.0.0", self.http_port), StaticHandler)
        if self.http_port == 0:
            self.http_port = self.httpd.server_address[1]
        print(f"[*] Web teleop dashboard started: http://0.0.0.0:{self.http_port} (Local: http://localhost:{self.http_port})")
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()

    def stop_http_server(self):
        """Stop static web server"""
        if self.httpd:
            try:
                self.httpd.shutdown()
                self.httpd.server_close()
            except Exception:
                pass
            self.httpd = None

    async def run_ws_server(self):
        """Start WebSocket server (Asyncio)"""
        self.ws_loop = asyncio.get_running_loop()
        self.stop_event = asyncio.Event()
        print(f"[*] WebSocket control service listening on: ws://0.0.0.0:{self.ws_port}")
        self.ws_server = await ws_serve(self.handle_ws_client, "0.0.0.0", self.ws_port)
        if self.ws_port == 0 and hasattr(self.ws_server, "sockets") and self.ws_server.sockets:
            self.ws_port = self.ws_server.sockets[0].getsockname()[1]
        try:
            await self.stop_event.wait()
        except asyncio.CancelledError:
            pass
        finally:
            self.ws_server.close()
            await self.ws_server.wait_closed()

    def stop(self):
        """Stop all services cleanly"""
        self.running = False
        self.stop_http_server()
        if self.bridge:
            self.bridge.close()
        if self.ws_loop:
            if hasattr(self, "stop_event") and self.stop_event:
                try:
                    self.ws_loop.call_soon_threadsafe(self.stop_event.set)
                except Exception:
                    pass
            elif self.ws_server:
                try:
                    self.ws_loop.call_soon_threadsafe(self.ws_server.close)
                except Exception:
                    pass

def get_local_ip():
    """Get local LAN IP address"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

def main():
    parser = argparse.ArgumentParser(description="MakeYourPet Phone A Teleoperation Relay Server")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT, help="Web dashboard HTTP port (default: 8080)")
    parser.add_argument("--ws-port", type=int, default=DEFAULT_WS_PORT, help="Control WebSocket port (default: 8081)")
    parser.add_argument("--chica-host", type=str, default=DEFAULT_CHICA_HOST, help="Phone B (Chica) IP address (default: 127.0.0.1)")
    parser.add_argument("--chica-port", type=int, default=DEFAULT_CHICA_PORT, help="Phone B (Chica) TCP port (default: 18711)")
    parser.add_argument("--mock", action="store_true", help="Enable mock telemetry mode (test UI without robot)")
    args = parser.parse_args()

    web_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pilot_web")
    if not os.path.exists(web_dir):
        print(f"[Error] Web directory not found: {web_dir}")
        sys.exit(1)

    local_ip = get_local_ip()
    print("=" * 68)
    print("   MAKE YOUR PET - DUAL PHONE TELEOP & ONNX RELAY SERVER")
    print("=" * 68)
    print(f" > Local IP Address:  {local_ip}")
    print(f" > Phone A Browser:   http://{local_ip}:{args.http_port} or http://localhost:{args.http_port}")
    print(f" > Phone B (Chica):   {args.chica_host}:{args.chica_port} {'[MOCK MODE]' if args.mock else ''}")
    print("=" * 68)

    manager = PilotServerManager(
        web_dir=web_dir,
        http_port=args.http_port,
        ws_port=args.ws_port,
        chica_host=args.chica_host,
        chica_port=args.chica_port,
        mock=args.mock
    )

    manager.start_http_server()

    try:
        asyncio.run(manager.run_ws_server())
    except KeyboardInterrupt:
        print("\n[*] Shutting down relay server...")
        manager.stop()

if __name__ == "__main__":
    main()
