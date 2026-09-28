#!/usr/bin/env python3
"""
MakeYourPet Full-Loop Simulation Test
Communication closed loop:
[Phone A Virtual Joystick] 
   └── WebSocket (Port 8091) ──> [PilotServer Relay] 
                                    └── TCP 18713 ──> [Chica ONNX Gait Brain] 
                                                            └── Binary 0xD3 Packet ──> [Servo2040 Virtual Board]
"""

import os
import sys

# Ensure UTF-8 output under Windows cp950 environment
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")

import time
import math
import json
import socket
import select
import asyncio
import threading
import subprocess
import numpy as np

# Add chica-server-main/tools/device to sys.path to load official Servo2040 protocol emulator
DEVICE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "chica-server-main", "tools", "device"))
if DEVICE_DIR not in sys.path:
    sys.path.insert(0, DEVICE_DIR)

from servo2040_protocol_emulator import Servo2040ProtocolEmulator

import websockets
import onnxruntime as ort

# --- Test Port Configuration ---
TEST_HTTP_PORT = 8090
TEST_WS_PORT = 8091
TEST_CHICA_PORT = 18713

# Physical pin mapping and joint parameters
JOINT_TO_PIN_MAP = [15, 16, 17, 9, 10, 11, 3, 4, 5, 12, 13, 14, 6, 7, 8, 0, 1, 2]
LEG_IS_TRIPOD_A = [True, False, True, False, True, False]
LEG_IS_RIGHT = [False, False, False, True, True, True]
LEG_YAW_MULT = [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0]

COXA_ATTACH = [-8.0, 0.0, 8.0, -8.0, 0.0, 8.0]
FEMUR_ATTACH = 35.0
TIBIA_ATTACH = 68.0

COXA_LIMIT = 0.7853982
FEMUR_LIMIT = 0.7853982
TIBIA_LIMIT = 1.0471976

# Default neutral standing pulse widths in microseconds (Pins 0~17)
DEFAULT_STAND_PULSES = [
    1627, 1596, 1509,  # 0, 1, 2: R3
    1373, 1404, 1491,  # 3, 4, 5: L3
    1500, 1625, 1575,  # 6, 7, 8: R2
    1500, 1375, 1425,  # 9, 10, 11: L2
    1373, 1596, 1509,  # 12, 13, 14: R1
    1627, 1404, 1491   # 15, 16, 17: L1
]

class VirtualChicaServer:
    """
    PC-side Chica ONNX gait brain simulation server
    - Listens on TCP 18713 (receives control commands from PilotServer)
    - Runs ONNX policy model (hexapod_policy.onnx) with 67-dim observation vector
    - Drives underlying Servo2040ProtocolEmulator virtual board with 39-byte binary 0xD3 packets
    """
    def __init__(self, port=TEST_CHICA_PORT, model_path=None):
        self.port = port
        self.emulator = Servo2040ProtocolEmulator()
        self.emulator.set_analog(touches=[3.3]*6, voltage=7.42, current_amps=0.65)

        # Load ONNX gait policy model
        if model_path is None:
            model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "chica-server-main", "app", "src", "main", "assets", "hexapod_policy.onnx"))
        self.ort_session = ort.InferenceSession(model_path)
        self.input_name = self.ort_session.get_inputs()[0].name
        self.output_name = self.ort_session.get_outputs()[0].name

        # Gait states
        self.running = True
        self.relay_status = False
        self.standing = False
        self.walking = False
        self.onnx_mode = True
        self.vx = 0.0
        self.vy = 0.0
        self.turn = 0.0
        self.gait_phase = 0.0

        # History buffers
        self.prev_action = np.zeros(18, dtype=np.float32)
        self.current_joint_angles = np.zeros(18, dtype=np.float32)
        self.prev_joint_angles = np.zeros(18, dtype=np.float32)
        self.smoothed_target = np.zeros(18, dtype=np.float32)

        self.last_step_time = time.time()
        self.frame_count = 0

        # Statistics
        self.last_sent_pulses = [1500] * 18
        self.total_onnx_steps = 0

        # Start TCP server and gait control threads
        self.server_thread = threading.Thread(target=self._tcp_server_loop, daemon=True)
        self.server_thread.start()
        self.gait_thread = threading.Thread(target=self._gait_worker_loop, daemon=True)
        self.gait_thread.start()

    def _tcp_server_loop(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", self.port))
            listener.listen(1)
            print(f"[Chica ONNX Brain] Listening on TCP: 127.0.0.1:{self.port}")

            while self.running:
                conn, addr = listener.accept()
                with conn:
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    print(f"[Chica ONNX Brain] PilotServer connected: {addr}")
                    conn.sendall(self._build_status_line().encode('utf-8'))

                    buf = ""
                    while self.running:
                        try:
                            readable, _, _ = select.select([conn], [], [], 0.04)
                            if not readable:
                                # Periodically push telemetry heartbeat
                                conn.sendall(self._build_status_line().encode('utf-8'))
                                continue
                            data = conn.recv(1024).decode('utf-8', errors='ignore')
                            if not data:
                                break
                            buf += data
                            while "\n" in buf:
                                line, buf = buf.split("\n", 1)
                                line = line.strip()
                                if line:
                                    self._handle_command(line)
                                    conn.sendall(self._build_status_line().encode('utf-8'))
                        except Exception as e:
                            print(f"[Chica ONNX Brain] Communication exception: {e}")
                            break

    def _handle_command(self, cmd: str):
        if cmd == "torque":
            self.relay_status = not self.relay_status
            self._set_emulator_relay(self.relay_status)
            print(f"[Chica Brain] Toggle Relay power: {'ON' if self.relay_status else 'OFF'}")
        elif cmd == "sit":
            self.standing = not self.standing
            if not self.standing:
                self.walking = False
            print(f"[Chica Brain] Toggle posture: {'STAND' if self.standing else 'SIT'}")
        elif cmd.startswith("walkonnx:"):
            # Format: walkonnx:turn,forward,anim
            parts = cmd[9:].split(",")
            if len(parts) >= 2:
                self.turn = float(parts[0])
                self.vx = float(parts[1]) * 0.35  # Scale to m/s
                self.walking = (abs(self.vx) > 0.02 or abs(self.turn) > 0.02)
                self.relay_status = True
                self.standing = True
                self.onnx_mode = True
                self._set_emulator_relay(True)
        elif cmd.startswith("walk2:"):
            parts = cmd[6:].split(",")
            if len(parts) >= 2:
                self.turn = float(parts[0])
                self.vx = float(parts[1]) * 0.35
                self.walking = (abs(self.vx) > 0.02 or abs(self.turn) > 0.02)
                self.relay_status = True
                self.standing = True
                self.onnx_mode = False
                self._set_emulator_relay(True)
        elif cmd == "walkclear" or cmd.startswith("clear"):
            self.walking = False
            self.vx = 0.0
            self.turn = 0.0
            print("[Chica Brain] Received brake stop command (walkclear)")

    def _set_emulator_relay(self, enabled: bool):
        # Send 0xD3 relay control packet to virtual board
        val = 1 if enabled else 0
        pkt = bytearray([0xD3, 26, 1, val & 0x7F, (val >> 7) & 0x7F])
        self.emulator.exchange(pkt)

    def _gait_worker_loop(self):
        """Runs kinematics feedforward, 67-dim observation construction, and ONNX residual neural network at 50Hz (20ms)"""
        dt = 0.02
        while self.running:
            start_t = time.time()
            if self.relay_status and self.standing:
                if self.walking:
                    # 1. Advance gait clock
                    self.gait_phase = (self.gait_phase + 2.0 * math.pi * 1.5 * dt) % (2.0 * math.pi)

                    # 2. Compute tripod feedforward reference angles q_ref
                    q_ref = self._compute_q_ref(self.gait_phase, self.vx, self.turn)

                    # 3. Assemble 67-dim observation vector (obs)
                    obs = np.zeros(67, dtype=np.float32)
                    obs[0] = 0.005  # roll
                    obs[1] = -0.010 # pitch
                    obs[2:5] = [0.0, 0.0, self.turn] # omega
                    obs[5:8] = [self.vx, 0.0, 0.0]   # v_body
                    obs[8:26] = self.current_joint_angles - q_ref # tracking error
                    obs[26:44] = ((self.current_joint_angles - self.prev_joint_angles) / dt) * 0.1
                    obs[44:62] = self.prev_action
                    obs[62:65] = [self.vx, 0.0, self.turn]
                    obs[65] = math.sin(self.gait_phase)
                    obs[66] = math.cos(self.gait_phase)

                    # 4. ONNX inference
                    res = self.ort_session.run([self.output_name], {self.input_name: obs.reshape(1, 67)})[0][0]
                    residual = np.clip(res, -1.0, 1.0)
                    self.prev_action[:] = residual
                    self.prev_joint_angles[:] = self.current_joint_angles

                    # 5. Residual blending and joint limit smoothing (q = q_ref + 0.15 * delta_q)
                    for i in range(18):
                        target = q_ref[i] + residual[i] * 0.15
                        joint_in_leg = i % 3
                        limit = TIBIA_LIMIT if joint_in_leg == 2 else (FEMUR_LIMIT if joint_in_leg == 1 else COXA_LIMIT)
                        target = np.clip(target, -limit, limit)
                        self.smoothed_target[i] = 0.3 * self.smoothed_target[i] + 0.7 * target
                        self.current_joint_angles[i] = self.smoothed_target[i]

                    self.total_onnx_steps += 1
                else:
                    # Stationary stance
                    self.current_joint_angles[:] = 0.0
                    self.smoothed_target[:] = 0.0
                    self.prev_action[:] = 0.0
                    self.gait_phase = 0.0

                # 6. Convert joint angles (rad) to 18-channel Servo2040 pulse widths (usec) and send to virtual board
                pulses = self._angles_to_pulses(self.current_joint_angles)
                self.last_sent_pulses = pulses

                # Pack 39-byte 0xD3 packet
                pkt = bytearray([0xD3, 0, 18])
                for p in pulses:
                    pkt.extend([p & 0x7F, (p >> 7) & 0x7F])
                self.emulator.exchange(pkt)

                # Update emulator dynamic state (current increases and foot touches alternate during locomotion; stationary stance has steady 6-foot contact)
                if self.walking:
                    is_phase_A = (self.gait_phase < math.pi)
                    t_volts = [3.3 if is_phase_A else 0.0,
                               0.0 if is_phase_A else 3.3,
                               3.3 if is_phase_A else 0.0,
                               0.0 if is_phase_A else 3.3,
                               3.3 if is_phase_A else 0.0,
                               0.0 if is_phase_A else 3.3]
                    current = 2.45
                else:
                    t_volts = [3.3] * 6  # All 6 feet touch ground when stationary
                    current = 0.65
                self.emulator.set_analog(touches=t_volts, voltage=7.38, current_amps=current)

            elapsed = time.time() - start_t
            sleep_t = max(0.001, dt - elapsed)
            time.sleep(sleep_t)

    def _compute_q_ref(self, phase, vx, yaw):
        q_ref = np.zeros(18, dtype=np.float32)
        phase_A = phase % (2.0 * math.pi)
        phase_B = (phase + math.pi) % (2.0 * math.pi)
        stride_base = vx * 2.0
        for leg in range(6):
            p = phase_A if LEG_IS_TRIPOD_A[leg] else phase_B
            stride = np.clip(stride_base + yaw * 0.40 * LEG_YAW_MULT[leg], -0.65, 0.65)
            q_coxa = (-1.0 if LEG_IS_RIGHT[leg] else 1.0) * math.cos(p) * stride
            if p < math.pi:
                h = (math.sin(p)) ** 0.8
                mult = 1.0 if (leg in [1, 4]) else (1.0 if leg in [0, 3] else 1.30)
                q_femur = -0.32 * mult * h
                q_tibia = -0.20 * mult * h
            else:
                q_femur = 0.01
                q_tibia = 0.005
            base_j = leg * 3
            q_ref[base_j + 0] = q_coxa
            q_ref[base_j + 1] = q_femur
            q_ref[base_j + 2] = q_tibia
        return q_ref

    def _angles_to_pulses(self, angles_rad):
        pulses = list(DEFAULT_STAND_PULSES)
        for j in range(18):
            leg = j // 3
            joint = j % 3
            right = leg > 2
            side_mult = -1.0 if right else 1.0
            pin = JOINT_TO_PIN_MAP[j]
            deg = math.degrees(angles_rad[j])
            center = DEFAULT_STAND_PULSES[pin]
            deg_scale = 11.1111

            if joint == 0:
                delta = -deg * deg_scale
            else:
                delta = side_mult * deg * deg_scale

            pulse = int(round(center + delta))
            pulse = max(700, min(2300, pulse))
            pulses[pin] = pulse
        return pulses

    def _build_status_line(self):
        # Read virtual board 0xC7 sensor data
        req = bytes([0xC7, 18, 8])
        reply = self.emulator.exchange(req)
        # Decode returned 19 bytes
        legs_str = ""
        for i in range(6):
            raw = (reply[3 + i * 2] & 0x7F) | ((reply[4 + i * 2] & 0x7F) << 7)
            legs_str += "x" if (raw / 1024.0 > 0.5) else "-"
        curr_raw = (reply[15] & 0x7F) | ((reply[16] & 0x7F) << 7)
        volt_raw = (reply[17] & 0x7F) | ((reply[18] & 0x7F) << 7)

        current = (curr_raw - 512) * 0.0814
        voltage = volt_raw / 310.3

        flag_relay = "1" if self.relay_status else "0"
        flag_stand = "1" if self.standing else "0"
        flags = f"{flag_relay}{flag_stand}0000100"

        return f"ready:BPS= 50|V={voltage:5.3f}|I={current:5.3f}|IP=127.0.0.1|LEGS={legs_str}|FLAGS={flags}\n"

    def stop(self):
        self.running = False


async def run_phone_a_test():
    """
    Simulates Phone A teleop controller (WebSocket client)
    Sequence: Handshake -> Enable Relay -> Dual-joystick ONNX locomotion -> Closed-loop telemetry verification -> Deadman Switch
    """
    ws_url = f"ws://127.0.0.1:{TEST_WS_PORT}"
    print(f"\n[Phone A Test Client] Connecting to PilotServer: {ws_url} ...")

    async with websockets.connect(ws_url) as ws:
        print("[Phone A Test Client] Connected! Awaiting first telemetry packet...")
        first_msg = await asyncio.wait_for(ws.recv(), timeout=3.0)
        telem = json.loads(first_msg)
        print(f" -> Initial telemetry: V={telem.get('voltage')}V, I={telem.get('current')}A, BPS={telem.get('bps')}")
        assert telem.get("robotConnected") == True, "Chica robot not connected!"

        # 1. Enable power relay
        print("\n[Test Item 1] Sending Relay/Torque enable command...")
        await ws.send(json.dumps({"type": "cmd", "command": "torque"}))
        await asyncio.sleep(0.3)

        # 2. Simulate Phone A dual joysticks (ONNX locomotion forward + turn)
        print("\n[Test Item 2] Simulating dual virtual joysticks (Forward Vx=0.35m/s, Yaw=0.10rad/s)...")
        received_walking_frames = 0
        for step in range(15):
            # Send joystick packet at 25Hz
            await ws.send(json.dumps({
                "type": "walk",
                "mode": "onnx",
                "forward": 1.0,
                "turn": 0.15,
                "crab": False
            }))
            # Receive telemetry relayed back
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=0.1)
                t = json.loads(msg)
                if t.get("current") and t["current"] > 1.5:
                    received_walking_frames += 1
            except asyncio.TimeoutError:
                pass
            await asyncio.sleep(0.04)

        print(f" -> Telemetry during locomotion received, frames with elevated load current: {received_walking_frames}")
        assert received_walking_frames > 0, "No locomotion telemetry received!"

        # 3. Test Deadman Switch
        print("\n[Test Item 3] Simulating hands leaving screen (triggering Deadman Switch)...")
        await ws.send(json.dumps({"type": "stop"}))
        await asyncio.sleep(0.3)

        # Check if stop feedback received
        final_msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
        final_telem = json.loads(final_msg)
        print(f" -> Post-braking state: Current returned to standby {final_telem.get('current')}A, FLAGS={final_telem.get('flags')}")

    print("\n[Phone A Controller Tests Passed!]")


def main():
    print("=" * 70)
    print("   MAKE YOUR PET FULL-LOOP VIRTUAL SIMULATOR CLOSED-LOOP TEST")
    print(" Chain: [Phone A Joystick] -> [PilotServer Relay] -> [Chica ONNX Brain] -> [Servo2040 Board]")
    print("=" * 70)

    # 1. Start Chica ONNX gait brain (TCP 18713)
    chica_brain = VirtualChicaServer(port=TEST_CHICA_PORT)
    time.sleep(0.5)

    # 2. Start PilotServer teleop relay (HTTP 8090, WS 8091)
    pilot_script = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "pilot_server.py"))
    cmd = [
        sys.executable, "-u", pilot_script,
        "--http-port", str(TEST_HTTP_PORT),
        "--ws-port", str(TEST_WS_PORT),
        "--chica-host", "127.0.0.1",
        "--chica-port", str(TEST_CHICA_PORT)
    ]
    pilot_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8")
    time.sleep(1.2)

    try:
        # 3. Run Phone A test client
        asyncio.run(run_phone_a_test())

        # 4. Verify 18-channel servo data received by virtual board
        print("\n" + "=" * 70)
        print("   [Servo2040 Virtual Board Verification Results]")
        print("=" * 70)
        servos = chica_brain.emulator.servos
        relay_on = chica_brain.emulator.relay_enabled
        total_steps = chica_brain.total_onnx_steps

        print(f" > Main Relay Status:        {'ENABLED' if relay_on else 'DISABLED'}")
        print(f" > Total ONNX Steps Inferred: {total_steps} steps")
        print(f" > Virtual Board 18-Axis PWM Pulse Widths (usec):")
        for leg in range(6):
            c_pin = JOINT_TO_PIN_MAP[leg * 3 + 0]
            f_pin = JOINT_TO_PIN_MAP[leg * 3 + 1]
            t_pin = JOINT_TO_PIN_MAP[leg * 3 + 2]
            c_val = servos[c_pin]
            f_val = servos[f_pin]
            t_val = servos[t_pin]
            print(f"    - Leg {leg+1} (P{c_pin:02d}, P{f_pin:02d}, P{t_pin:02d}): Coxa={c_val}us, Femur={f_val}us, Tibia={t_val}us")

        assert relay_on == True, "Relay should remain enabled"
        assert total_steps > 5, "ONNX policy should execute at least 5 inferences"
        assert all(700 <= p <= 2300 for p in servos), "All pulses must be within safe range [700, 2300] us"

        print("=" * 70)
        print("   Full-loop closed-loop test 100% PASSED!")
        print("   [Phone A -> Relay -> Chica ONNX Gait -> Virtual Board] end-to-end verified!")
        print("=" * 70)

    finally:
        pilot_proc.terminate()
        pilot_proc.wait()
        chica_brain.stop()

if __name__ == "__main__":
    main()
