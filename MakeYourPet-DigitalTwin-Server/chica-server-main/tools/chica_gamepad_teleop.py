#!/usr/bin/env python3
"""
ChicaServer Gamepad Teleop Controller
=====================================
Supported: PS4 / PS5 (DualSense) / Xbox / Switch Pro / Logitech / generic PC Bluetooth or USB gamepad

Mechanism:
  [Gamepad] -> (Bluetooth/USB) -> [This script (reads analog axes & buttons)] -> (Wi-Fi TCP 18711) -> [Hexapod Robot ChicaServer]

Button Mapping (Standard Gamepad Layout):
  - Left Stick: Pitch = Forward/Backward; Roll = Lateral crab walk (if enabled)
  - Right Stick: Yaw = In-place turning
  - A / Cross: Toggle Stand / Sit (sit)
  - Y / Triangle: Toggle Servo Power Relay (torque)
  - B / Circle: Brake stop (walkclear)
  - X / Square: Posture calibration / level maintain (level)
  - LB / L1: Precision crawl mode (max speed 0.3)
  - RB / R1: Sprint mode (max speed 1.0)
  - D-Pad: Fine-tune body height and CG (setxy / setzu)
"""

import sys
import time
import socket
import argparse
import threading

# Suppress Pygame welcome message
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame


class ChicaGamepadTeleop:
    def __init__(self, host: str, port: int = 18711, deadzone: float = 0.12, rate_hz: float = 25.0):
        self.host = host
        self.port = port
        self.deadzone = deadzone
        self.interval = 1.0 / rate_hz
        self.sock = None
        self.running = True
        self.is_walking = False
        self.max_speed = 0.6  # Default normal speed (0.3=crawl, 0.6=normal, 1.0=sprint)
        
        # Telemetry cache
        self.telemetry = {
            "V": "---",
            "I": "---",
            "BPS": "0",
            "FLAGS": "---------",
            "RAW": ""
        }

    def connect(self):
        print(f"\n[Connecting] Connecting to robot ChicaServer [{self.host}:{self.port}]...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(3.0)
        try:
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.5)
            # Read initial status line
            first_line = self.sock.recv(1024).decode('utf-8', errors='ignore')
            self._parse_status(first_line)
            print(f"[Connected] Hardware status: {self.telemetry['RAW'].strip()}")
            
            # Start background telemetry receiving thread
            t = threading.Thread(target=self._recv_loop, daemon=True)
            t.start()
            return True
        except Exception as e:
            print(f"[Error] Connection failed: {e}")
            print(f"   Please make sure the phone and PC are on the same Wi-Fi and ChicaServer is running on the phone!")
            return False

    def _recv_loop(self):
        buf = ""
        while self.running and self.sock:
            try:
                chunk = self.sock.recv(1024).decode('utf-8', errors='ignore')
                if not chunk:
                    break
                buf += chunk
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    self._parse_status(line.strip())
            except socket.timeout:
                continue
            except Exception:
                break

    def _parse_status(self, line: str):
        if not line:
            return
        self.telemetry["RAW"] = line
        parts = line.split('|')
        for p in parts:
            if p.startswith("V="):
                self.telemetry["V"] = p.split('=')[1]
            elif p.startswith("I="):
                self.telemetry["I"] = p.split('=')[1]
            elif p.startswith("BPS="):
                self.telemetry["BPS"] = p.split('=')[1].strip()
            elif p.startswith("FLAGS="):
                self.telemetry["FLAGS"] = p.split('=')[1].strip()

    def send_cmd(self, cmd: str):
        if not self.sock:
            return
        try:
            self.sock.sendall((cmd + "\n").encode('utf-8'))
        except Exception as e:
            print(f"\n[Warning] Failed to send command: {e}")

    def apply_deadzone(self, val: float) -> float:
        if abs(val) < self.deadzone:
            return 0.0
        # Smooth transition deadzone
        sign = 1.0 if val > 0 else -1.0
        scaled = (abs(val) - self.deadzone) / (1.0 - self.deadzone)
        return sign * scaled

    def run(self):
        pygame.init()
        pygame.joystick.init()

        num_joysticks = pygame.joystick.get_count()
        if num_joysticks == 0:
            print("\n[Error] No gamepad detected!")
            print("   Please connect your gamepad (Xbox / PS4 / PS5 / Switch Pro) via USB or Bluetooth and retry.")
            return

        js = pygame.joystick.Joystick(0)
        js.init()
        print(f"\n[Gamepad] Detected controller: [{js.get_name()}]")
        print(f"   Axes: {js.get_numaxes()} | Buttons: {js.get_numbuttons()} | Hats (D-Pad): {js.get_numhats()}")
        
        if not self.connect():
            return

        print("\n" + "="*60)
        print("[Gamepad] Teleop ready! (Press Ctrl+C to exit anytime)")
        print("   - Left Stick Pitch: Forward / Backward")
        print("   - Right Stick Yaw: Turn in-place")
        print("   - A / Cross button: Stand / Sit toggle")
        print("   - Y / Triangle: Servo power toggle (Torque)")
        print("   - B / Circle: Emergency stop (walkclear)")
        print("   - LB / RB: Slow mode (0.3) / Sprint mode (1.0)")
        print("="*60 + "\n")

        last_walk_cmd = ""
        zero_count = 0

        try:
            while self.running:
                loop_start = time.time()
                pygame.event.pump()

                # --- 1. Button event processing ---
                # A / Cross (Button 0) -> Stand / Sit
                if js.get_button(0):
                    print(">> [Button A/Cross] Toggle Stand/Sit (sit)")
                    self.send_cmd("sit")
                    time.sleep(0.3)

                # B / Circle (Button 1) -> Brake stop
                elif js.get_button(1):
                    print(">> [Button B/Circle] Emergency stop (walkclear)")
                    self.send_cmd("walkclear")
                    self.is_walking = False
                    time.sleep(0.3)

                # X / Square (Button 2) -> Auto level balance
                elif js.get_button(2):
                    print(">> [Button X/Square] Toggle active level maintaining (level)")
                    self.send_cmd("level")
                    time.sleep(0.3)

                # Y / Triangle (Button 3) -> Servo power relay
                elif js.get_button(3):
                    print(">> [Button Y/Triangle] Toggle servo power relay (torque)")
                    self.send_cmd("torque")
                    time.sleep(0.3)

                # LB (Shoulder button 4) -> Precision crawl mode
                if js.get_button(4):
                    self.max_speed = 0.3
                # RB (Shoulder button 5) -> High-speed sprint mode
                elif js.get_button(5):
                    self.max_speed = 1.0
                else:
                    self.max_speed = 0.6

                # --- 2. Analog joystick reading and processing ---
                # Standard axis definitions:
                # Axis 1: Left stick vertical (pushing up is typically negative, inverted)
                # Axis 0: Left stick horizontal
                # Axis 2 or 3: Right stick horizontal (depending on controller model)
                num_axes = js.get_numaxes()
                is_fpv = any(k in js.get_name().lower() for k in ["betafpv", "radio", "edgetx", "opentx", "taranis", "elrs", "frsky"])
                raw_pitch = js.get_axis(1) if num_axes > 1 else 0.0
                raw_fwd = raw_pitch if is_fpv else -raw_pitch
                
                # Compatibility for right stick axis index across controllers
                if num_axes >= 4:
                    raw_turn = js.get_axis(2) if abs(js.get_axis(2)) > abs(js.get_axis(3)) else js.get_axis(3)
                elif num_axes >= 3:
                    raw_turn = js.get_axis(2)
                elif num_axes >= 1:
                    raw_turn = js.get_axis(0)
                else:
                    raw_turn = 0.0

                forward = self.apply_deadzone(raw_fwd) * self.max_speed
                turn = self.apply_deadzone(raw_turn) * self.max_speed

                # Clamp to [-1.0, 1.0]
                forward = max(-1.0, min(1.0, forward))
                turn = max(-1.0, min(1.0, turn))

                # --- 3. Send locomotion command (Deadman Switch protection) ---
                if abs(forward) > 0.01 or abs(turn) > 0.01:
                    # When stick is engaged, send walk command at 25Hz
                    # Format: walk2:<turn>,<forward>,<anim>
                    cmd = f"walk2:{turn:.2f},{forward:.2f},0"
                    self.send_cmd(cmd)
                    last_walk_cmd = cmd
                    self.is_walking = True
                    zero_count = 0
                else:
                    # Stick back to center
                    if self.is_walking:
                        zero_count += 1
                        if zero_count >= 2:
                            # Two consecutive zero frames trigger immediate stop
                            self.send_cmd("walkclear")
                            self.is_walking = False
                            zero_count = 0

                # --- 4. Terminal dynamic HUD refresh ---
                flags = self.telemetry["FLAGS"]
                relay_on = flags[0] == '1' if len(flags) > 0 else False
                standing = flags[1] == '1' if len(flags) > 1 else False
                status_str = "STAND" if standing else "SIT"
                power_str = "POWERED" if relay_on else "UNPOWERED"

                hud = (
                    f"\r[BATT] Voltage: {self.telemetry['V']}V | "
                    f"Current: {self.telemetry['I']}A | "
                    f"Heartbeat: {self.telemetry['BPS']} BPS | "
                    f"Pose: {status_str}({power_str}) | "
                    f"Speed: {self.max_speed*100:.0f}% | "
                    f"Fwd: {forward:+.2f} Turn: {turn:+.2f} "
                )
                sys.stdout.write(hud)
                sys.stdout.flush()

                # Loop rate control
                elapsed = time.time() - loop_start
                sleep_time = max(0.001, self.interval - elapsed)
                time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\n[Stopping] Exiting...")
            self.send_cmd("walkclear")
            time.sleep(0.2)
            self.send_cmd("sit")
            time.sleep(0.5)
            self.send_cmd("bye")
        finally:
            self.running = False
            if self.sock:
                self.sock.close()
            pygame.quit()
            print("[Disconnected] Connection closed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ChicaServer Gamepad Teleop Controller")
    parser.add_argument("--ip", type=str, required=True, help="Robot onboard phone Wi-Fi IP (displayed on top of the App)")
    parser.add_argument("--port", type=int, default=18711, help="TCP port (default: 18711)")
    parser.add_argument("--deadzone", type=float, default=0.12, help="Joystick deadzone threshold (default: 0.12)")
    parser.add_argument("--rate", type=float, default=25.0, help="Command send rate in Hz (default: 25Hz)")
    args = parser.parse_args()

    teleop = ChicaGamepadTeleop(host=args.ip, port=args.port, deadzone=args.deadzone, rate_hz=args.rate)
    teleop.run()
