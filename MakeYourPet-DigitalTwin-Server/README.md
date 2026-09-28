# Make Your Pet Hexapod - Dual-Phone AI Locomotion System

> [!WARNING]
> **Experimental Project & Upstream Notice / 專案聲明與實測提醒**:
> - This server implementation is built upon [eternalnitrous/chica-server](https://github.com/eternalnitrous/chica-server).
> - **Hardware Status**: This system has undergone virtual closed-loop simulation and software emulation, but **has not yet been validated on physical hardware** (尚未通過實體測試).
> 
> 本伺服器系統架構是基於 [eternalnitrous/chica-server](https://github.com/eternalnitrous/chica-server) 進行建構與二次開發。目前已完成全鏈路虛擬閉環模擬測試，**尚未通過實體硬體測試**，實機部署時請注意安全防護與限流保護。

This repository hosts the **Dual-Phone AI Locomotion & Digital Twin Architecture** for the Make Your Pet Hexapod robot. It bridges edge reinforcement learning (PPO policy via ONNX Runtime Mobile) with real-time web telemetry and virtual hardware emulation.

---

## 1. System Architecture Overview

The system operates across three primary nodes:

```
[ Phone A: Operator Remote ] 
       │ (Touch Joysticks, HUD Telemetry)
       ▼ WebSocket (Port 8081) / HTTP (Port 8080)
[ PilotServer Relay (PC / Edge Gateway) ]
       │ TCP Commands (Port 18711) & Telemetry Stream
       ▼
[ Phone B: Robot Brain (chica-server-main) ]
       │ • 67-dim Observation Builder (IMU + Kinematics + Clocks)
       │ • ONNX Runtime Mobile (hexapod_policy.onnx)
       │ • 18-dim Residual Gait Fusion (q = q_ref + 0.15 * delta_q)
       │ • Zero-Allocation 50 Hz PWM Pulse Converter
       ▼ Binary 0xD3 / 0xC7 Packets (115200 baud USB Serial)
[ Pimoroni Servo 2040 Board / Virtual Emulator ]
       │ 18-channel PWM Pulses & ADC Sensors (Voltage, Current, Foot Contacts)
```

---

## 2. Key Features Implemented So Far

### Feature 1: Embedded ONNX Locomotion Engine (`chica-server-main`)
* **ONNX Runtime Mobile Integration**: Integrated `com.microsoft.onnxruntime:onnxruntime-android:1.17.1` into the Android Chica server.
* **Self-Contained Model Packaging**: Embedded `hexapod_policy.onnx` (single 352 KB binary with embedded weights) directly into the APK assets.
* **67-Dimensional Observation Vector**: Real-time state assembling matching the MuJoCo `hexapod_env.py` specification:
  * Body orientation (Roll, Pitch) & angular velocities $(\omega_x, \omega_y, \omega_z)$
  * Body linear velocities $(v_x, v_y, v_z)$
  * 18 joint tracking errors $(q_{\text{current}} - q_{\text{ref}})$
  * 18 scaled joint angular velocities $(\dot{q} \times 0.1)$
  * 18 previous residual actions
  * 3 commanded velocities $(v_x, v_y, \omega_{\text{yaw}})$
  * 2 gait phase clock signals $(\sin\phi, \cos\phi)$
* **18-DOF Residual Gait Fusion**: Fuses analytical tripod kinematics with neural network residual offsets ($q = q_{\text{ref}} + 0.15 \cdot \Delta q$), applying joint limits (Coxa $\pm 45^\circ$, Femur $\pm 45^\circ$, Tibia $\pm 60^\circ$) and exponential moving average (EMA) smoothing.
* **Protocol Interception**: Added native support for `walkonnx:`, `walkai:`, and `onnx` toggle commands while maintaining full backward compatibility with original Chica text commands (`walk2:`, `torque`, `sit`, `stand`).

### Feature 2: Tactical Web Controller & Telemetry HUD (`pilot_web/`)
* **Dual Virtual Joysticks**: Real-time analog touch controls (Left: Forward/Strafe, Right: Yaw rotation) with customizable deadzones and return-to-center springs.
* **Hardware Deadman Switch**: Releasing the joysticks automatically transmits `walkclear` within 50 ms, stopping the robot immediately if the operator lets go.
* **Live Status HUD**: Real-time indicators for battery voltage (V), current draw (A), communication rate (BPS), torque relay state, and an interactive 6-foot contact matrix visualizing tripod alternation.
* **Asynchronous Pilot Relay Server (`pilot_server.py`)**: Built on Python `asyncio` + `websockets` + HTTP static serving, featuring a 350 ms safety watchdog that cuts motor commands upon connection loss.

### Feature 3: Full-Loop Virtual Closed-Loop Simulator (`test_full_loop_simulation.py`)
* **Hardware-Free End-to-End Testing**: Validates the entire pipeline (`Phone A Web HUD -> PilotServer -> Chica ONNX Brain -> Virtual Servo2040 Board`) on a standard PC without requiring a physical robot.
* **Protocol Emulation**: Integrates the official `Servo2040ProtocolEmulator` to process 39-byte `0xD3` (SET PWM) packets and simulate `0xC7` (GET analog telemetry) responses with realistic dynamic load currents.

---

## 3. Issues Encountered & Solutions Applied

During code review and full-loop integration testing, several critical bugs and architectural challenges were identified and resolved:

### 1. Coordinate Frame & Mechanical Attach Angle Mismatch (Critical Bug)
* **The Problem**:  
  The initial pulse conversion code ported the legacy formula from Chica's C++ APK (`corrected -= femurAttach`, `corrected += tibiaAttach`). In the original APK, the inverse kinematics computed *global trigonometric angles* relative to the horizontal plane (Femur $\approx 46^\circ$, Tibia $\approx 61^\circ$).  
  However, in MuJoCo, the CAD parts are *already modeled with their attach angles* (Femur $35^\circ$, Tibia $68^\circ$), meaning $\theta = 0.0\text{ rad}$ represents the **nominal standing pose**.  
  Applying the legacy formula to relative angles shifted Femur to $1888\ \mu s$ / $1112\ \mu s$ and Tibia to $745\ \mu s$ / $2255\ \mu s$ (hitting physical servo stops and causing severe joint dislocation). Additionally, Left Coxa angles swung backwards while Right Coxa swung forwards, causing the robot to spin in circles instead of walking straight.
* **The Solution**:  
  Re-anchored the pulse generator around `DEFAULT_STAND_PULSES`:
  * **Coxa (Yaw)**: $\text{pulse} = \text{center} - \theta_{\text{deg}} \times 11.111$ (automatically accounts for symmetric forward swing across both sides).
  * **Femur & Tibia (Pitch)**: $\text{pulse} = \text{center} + (\text{isRight} ? -1.0 : 1.0) \times \theta_{\text{deg}} \times 11.111$ (compensates for mirrored physical servo mounting).  
  At $\theta = 0.0\text{ rad}$, all 18 servos now sit precisely at their calibrated neutral standing pulses ($1500\ \mu s$ safe zone).

### 2. Custom Pin Mapping & Calibration Overrides
* **The Problem**:  
  `JOINT_TO_PIN_MAP` was hardcoded, causing custom pin remappings in `config-2040.txt` (e.g., swapping a damaged pin channel) to be silently ignored during ONNX locomotion.
* **The Solution**:  
  Updated `OnnxLocomotionRunner` to dynamically check `servoCalibration.pin[leg][joint]` and user-calibrated center pulse ranges before falling back to default pin indices.

### 3. High-Frequency Garbage Collection Jitter (50 Hz ART GC Pressure)
* **The Problem**:  
  Calling `FloatBuffer.wrap()`, allocating new `float[]` arrays, and generating `Map` entries on every 20 ms cycle created significant memory churn. On mobile Android devices, frequent ART garbage collection pauses caused timing jitter and frame drops in the real-time loop.
* **The Solution**:  
  Introduced a **Zero-Allocation pipeline**: pre-allocated direct native `FloatBuffer` (`ByteBuffer.allocateDirect`), reusable observation/residual buffers, and in-place tensor rewinds, eliminating heap allocations during 50 Hz execution.

### 4. Abrupt Joint Jerk upon Stop Commands
* **The Problem**:  
  When receiving `walkclear` or when joystick inputs returned to zero, the gait phase and target angles were instantly zeroed, causing swinging legs to snap back to the standing pose in a single tick.
* **The Solution**:  
  Implemented soft deceleration decay: when `isMoving == false`, residual offsets decay by $0.85\times$ per tick and the EMA smoothing filter uses $\beta = 0.4$, smoothly setting the feet onto the ground within 150 ms.

### 5. Standalone ONNX Model Packaging
* **The Problem**:  
  The initial ONNX export used external weight files (`hexapod_policy.onnx.data`), which caused `context.getAssets().open()` in Android to throw `OrtException` because sub-asset file paths could not be resolved.
* **The Solution**:  
  Re-serialized the policy using ONNX's embedded weight format into a standalone single-file `hexapod_policy.onnx` (352 KB).

### 6. Static Foot Contact Indicator Blinking
* **The Problem**:  
  The virtual emulator alternated simulated foot contact touches even when the robot was standing still, causing the HUD leg contact indicators to blink continuously.
* **The Solution**:  
  Gated tripod phase alternation on `is_moving`; when stationary, all six feet report solid $3.3\text{ V}$ contact telemetry.

---

## 4. Verification & Testing

To run the automated full-loop simulation test:

```bash
# From workspace root
python MakeYourPet-DigitalTwin-Server/tools/test_full_loop_simulation.py
```

Expected output:
* PilotServer WebSocket and TCP 18713 communication online.
* Dual-joystick motion commands driving ONNX inference steps.
* Deadman switch triggering safe stop and torque verification.
* 18-channel virtual Servo2040 pulses verified 100% within the nominal standing range ($1373\ \mu s \sim 1627\ \mu s$).
