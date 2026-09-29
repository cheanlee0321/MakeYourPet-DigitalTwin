/**
 * MakeYourPet Pilot Web Controller
 * Phone A Touch Teleoperation Controller & Real-Time Tactical HUD
 */

(() => {
    // --- State Variables ---
    const state = {
        // Mode: 'onnx' or 'tripod'
        mode: 'onnx',
        speedGain: 1.0,
        crabMode: false,
        highClearance: false,
        isTorqueOn: false,
        isStanding: false,
        
        // Joystick input (normalized -1.0 ~ 1.0)
        leftJoy: { x: 0.0, y: 0.0, active: false, touchId: null },
        rightJoy: { x: 0.0, y: 0.0, active: false, touchId: null },

        // Telemetry data
        telemetry: {
            connected: false,
            robotIp: '0.0.0.0',
            voltage: 0.0,
            current: 0.0,
            bps: 0,
            legs: '------',
            flags: '000000000'
        },

        // Deadman switch state
        isMoving: false,
        lastSentStop: true
    };

    // --- DOM Element References ---
    const dom = {
        chicaStatus: document.getElementById('chica-status'),
        wsStatus: document.getElementById('ws-status'),
        btnModeOnnx: document.getElementById('btn-mode-onnx'),
        btnModeTripod: document.getElementById('btn-mode-tripod'),
        btnTorque: document.getElementById('btn-torque'),
        torqueText: document.getElementById('torque-text'),
        btnStand: document.getElementById('btn-stand'),
        standText: document.getElementById('stand-text'),
        btnEstop: document.getElementById('btn-estop'),

        // HUD data
        valVoltage: document.getElementById('val-voltage'),
        barVoltage: document.getElementById('bar-voltage'),
        voltZone: document.getElementById('volt-zone'),
        valCell: document.getElementById('val-cell'),

        valCurrent: document.getElementById('val-current'),
        barCurrent: document.getElementById('bar-current'),
        currZone: document.getElementById('curr-zone'),
        currWarnText: document.getElementById('curr-warn-text'),

        bpsTag: document.getElementById('bps-tag'),
        deadmanBadge: document.getElementById('deadman-indicator'),
        deadmanText: document.getElementById('deadman-text'),
        cmdStream: document.getElementById('cmd-stream'),

        // Hexapod leg nodes
        legs: {
            L1: document.getElementById('leg-L1'),
            L2: document.getElementById('leg-L2'),
            L3: document.getElementById('leg-L3'),
            R1: document.getElementById('leg-R1'),
            R2: document.getElementById('leg-R2'),
            R3: document.getElementById('leg-R3')
        },

        // Auxiliary switches
        btnCrab: document.getElementById('btn-crab'),
        crabText: document.getElementById('crab-text'),
        btnClearance: document.getElementById('btn-clearance'),
        clearanceText: document.getElementById('clearance-text'),
        btnCalibrate: document.getElementById('btn-calibrate'),
        speedSlider: document.getElementById('speed-slider'),
        speedVal: document.getElementById('speed-val'),

        // Joysticks
        leftCanvas: document.getElementById('joystick-left'),
        rightCanvas: document.getElementById('joystick-right'),
        leftReadout: document.getElementById('left-readout'),
        rightReadout: document.getElementById('right-readout')
    };

    // --- WebSocket Communication Module ---
    let ws = null;
    let wsReconnectTimer = null;
    const WS_PORT = 8081;

    function getWsUrl() {
        const host = window.location.hostname || '127.0.0.1';
        return `ws://${host}:${WS_PORT}`;
    }

    function initWebSocket() {
        if (wsReconnectTimer) clearTimeout(wsReconnectTimer);
        const url = getWsUrl();
        console.log(`Connecting to WebSocket: ${url}`);

        try {
            ws = new WebSocket(url);
        } catch (e) {
            console.error('WebSocket creation error:', e);
            scheduleReconnect();
            return;
        }

        ws.onopen = () => {
            console.log('WebSocket connected');
            updateBadge(dom.wsStatus, true, 'LINK: CONNECTED');
        };

        ws.onmessage = (event) => {
            try {
                const msg = JSON.parse(event.data);
                handleServerMessage(msg);
            } catch (err) {
                console.warn('Failed to parse WS msg:', event.data);
            }
        };

        ws.onerror = (err) => {
            console.warn('WebSocket error:', err);
        };

        ws.onclose = () => {
            console.log('WebSocket disconnected');
            updateBadge(dom.wsStatus, false, 'LINK: DISCONNECTED');
            updateBadge(dom.chicaStatus, false, 'ROBOT: OFFLINE');
            scheduleReconnect();
        };
    }

    function scheduleReconnect() {
        if (wsReconnectTimer) clearTimeout(wsReconnectTimer);
        wsReconnectTimer = setTimeout(initWebSocket, 2000);
    }

    function sendWs(payload) {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify(payload));
        }
    }

    function updateBadge(el, online, text) {
        if (!el) return;
        el.className = `badge ${online ? 'badge-online' : 'badge-offline'}`;
        const label = el.querySelector('.label');
        if (label) label.textContent = text;
    }

    // --- Server Messages & Telemetry Parsing ---
    function handleServerMessage(msg) {
        if (msg.type === 'telemetry') {
            state.telemetry.connected = msg.robotConnected;
            state.telemetry.robotIp = msg.ip || '0.0.0.0';
            state.telemetry.voltage = typeof msg.voltage === 'number' ? msg.voltage : NaN;
            state.telemetry.current = typeof msg.current === 'number' ? msg.current : NaN;
            state.telemetry.bps = msg.bps || 0;
            state.telemetry.legs = msg.legs || '------';
            state.telemetry.flags = msg.flags || '000000000';

            // Update robot connection status badge
            if (msg.robotConnected) {
                updateBadge(dom.chicaStatus, true, `ROBOT: ${state.telemetry.robotIp}`);
            } else {
                updateBadge(dom.chicaStatus, false, 'ROBOT: OFFLINE');
            }

            renderTelemetry();
        }
    }

    function renderTelemetry() {
        const t = state.telemetry;

        // 1. Voltage rendering
        if (!isNaN(t.voltage) && t.voltage > 0) {
            dom.valVoltage.textContent = t.voltage.toFixed(2);
            const cellV = (t.voltage / 2.0).toFixed(2);
            dom.valCell.textContent = `~ ${cellV}V / cell`;

            // 2S LiPo battery percentage (6.0V ~ 8.4V)
            const pct = Math.max(0, Math.min(100, ((t.voltage - 6.0) / 2.4) * 100));
            dom.barVoltage.style.width = `${pct}%`;

            if (t.voltage < 6.0) {
                dom.voltZone.className = 'zone-badge zone-alert';
                dom.voltZone.textContent = 'CUTOFF!';
            } else if (t.voltage < 6.4) {
                dom.voltZone.className = 'zone-badge zone-warn';
                dom.voltZone.textContent = 'LOW WARN';
            } else {
                dom.voltZone.className = 'zone-badge zone-ok';
                dom.voltZone.textContent = 'NORMAL';
            }
        } else {
            dom.valVoltage.textContent = '--.-';
            dom.valCell.textContent = '~ -.-V / cell';
            dom.barVoltage.style.width = '0%';
            dom.voltZone.className = 'zone-badge zone-ok';
            dom.voltZone.textContent = 'STANDBY';
        }

        // 2. Current rendering
        if (!isNaN(t.current)) {
            dom.valCurrent.textContent = t.current.toFixed(2);
            const currPct = Math.max(0, Math.min(100, (t.current / 10.0) * 100));
            dom.barCurrent.style.width = `${currPct}%`;

            if (t.current > 10.0) {
                dom.currZone.className = 'zone-badge zone-alert';
                dom.currZone.textContent = 'OVERLOAD';
                dom.currWarnText.textContent = 'CRITICAL CUTOFF!';
            } else if (t.current > 8.0) {
                dom.currZone.className = 'zone-badge zone-warn';
                dom.currZone.textContent = 'HIGH';
                dom.currWarnText.textContent = 'WARNING (>8A)';
            } else {
                dom.currZone.className = 'zone-badge zone-ok';
                dom.currZone.textContent = t.current > 1.5 ? 'WALKING' : 'IDLE';
                dom.currWarnText.textContent = 'NORMAL';
            }
        } else {
            dom.valCurrent.textContent = '--.-';
            dom.barCurrent.style.width = '0%';
            dom.currZone.className = 'zone-badge zone-ok';
            dom.currZone.textContent = 'STANDBY';
            dom.currWarnText.textContent = 'NORMAL';
        }

        // 3. Hexapod ground contact monitoring (LEGS)
        // Order: TS1=L1, TS2=L2, TS3=L3, TS4=R1, TS5=R2, TS6=R3
        const legNames = ['L1', 'L2', 'L3', 'R1', 'R2', 'R3'];
        for (let i = 0; i < 6; i++) {
            const char = t.legs.charAt(i);
            const isStance = char === 'x' || char === 'X';
            const node = dom.legs[legNames[i]];
            if (node) {
                if (isStance) {
                    node.className = 'leg-node stance';
                    node.querySelector('.leg-state').textContent = 'DOWN';
                } else {
                    node.className = 'leg-node swing';
                    node.querySelector('.leg-state').textContent = 'AIR';
                }
            }
        }

        // 4. BPS refresh
        dom.bpsTag.textContent = `${t.bps} BPS`;

        // 5. FLAGS status sync (Relay, Stand, and Crab)
        if (t.flags && t.flags.length >= 2) {
            state.isTorqueOn = t.flags.charAt(0) === '1';
            state.isStanding = t.flags.charAt(1) === '1';
            if (t.flags.length >= 4) {
                state.crabMode = t.flags.charAt(3) === '1';
            }
            updateButtonStates();
        }
    }

    function updateButtonStates() {
        if (state.isTorqueOn) {
            dom.btnTorque.classList.add('active');
            dom.torqueText.textContent = 'RELAY: ON';
        } else {
            dom.btnTorque.classList.remove('active');
            dom.torqueText.textContent = 'RELAY: OFF';
        }

        if (state.isStanding) {
            dom.btnStand.classList.add('standing');
            dom.standText.textContent = 'STAND';
        } else {
            dom.btnStand.classList.remove('standing');
            dom.standText.textContent = 'SIT';
        }

        if (dom.btnCrab && dom.crabText) {
            dom.btnCrab.classList.toggle('active', state.crabMode);
            dom.crabText.textContent = state.crabMode ? 'ON' : 'OFF';
        }
    }

    // --- Dual Virtual Joystick Canvas & Touch Engine ---
    class VirtualJoystick {
        constructor(canvas, onUpdate) {
            this.canvas = canvas;
            this.ctx = canvas.getContext('2d');
            this.onUpdate = onUpdate;
            this.size = canvas.width;
            this.center = this.size / 2;
            this.maxRadius = this.size * 0.38;
            this.knobRadius = this.size * 0.16;

            this.x = 0.0; // [-1.0, 1.0]
            this.y = 0.0; // [-1.0, 1.0]
            this.active = false;
            this.touchId = null;

            this.initEvents();
            this.render();
        }

        initEvents() {
            const el = this.canvas;

            // Touch events (Mobile)
            el.addEventListener('touchstart', (e) => {
                e.preventDefault();
                if (this.active) return;
                const touch = e.changedTouches[0];
                this.touchId = touch.identifier;
                this.active = true;
                this.handleMove(touch.clientX, touch.clientY);
            }, { passive: false });

            window.addEventListener('touchmove', (e) => {
                if (!this.active) return;
                for (let i = 0; i < e.changedTouches.length; i++) {
                    const touch = e.changedTouches[i];
                    if (touch.identifier === this.touchId) {
                        e.preventDefault();
                        this.handleMove(touch.clientX, touch.clientY);
                        break;
                    }
                }
            }, { passive: false });

            const touchEndHandler = (e) => {
                if (!this.active) return;
                for (let i = 0; i < e.changedTouches.length; i++) {
                    if (e.changedTouches[i].identifier === this.touchId) {
                        this.reset();
                        break;
                    }
                }
            };
            window.addEventListener('touchend', touchEndHandler);
            window.addEventListener('touchcancel', touchEndHandler);

            // Mouse events (PC browser fallback / debugging)
            let mouseDown = false;
            el.addEventListener('mousedown', (e) => {
                mouseDown = true;
                this.active = true;
                this.handleMove(e.clientX, e.clientY);
            });
            window.addEventListener('mousemove', (e) => {
                if (mouseDown && this.active) {
                    this.handleMove(e.clientX, e.clientY);
                }
            });
            window.addEventListener('mouseup', () => {
                if (mouseDown) {
                    mouseDown = false;
                    this.reset();
                }
            });
        }

        handleMove(clientX, clientY) {
            const rect = this.canvas.getBoundingClientRect();
            const posX = clientX - rect.left - this.center;
            const posY = clientY - rect.top - this.center;

            const dist = Math.hypot(posX, posY);
            const angle = Math.atan2(posY, posX);
            const clampedDist = Math.min(dist, this.maxRadius);

            // Normalize (-1.0 ~ 1.0)
            this.x = (clampedDist * Math.cos(angle)) / this.maxRadius;
            // Swiping up is positive velocity (MuJoCo/Chica: forward is positive X/Y)
            this.y = -(clampedDist * Math.sin(angle)) / this.maxRadius;

            // Deadzone filtering (Deadzone 0.05)
            if (Math.hypot(this.x, this.y) < 0.05) {
                this.x = 0.0;
                this.y = 0.0;
            }

            this.render();
            if (this.onUpdate) this.onUpdate(this.x, this.y, true);
        }

        reset() {
            this.active = false;
            this.touchId = null;
            this.x = 0.0;
            this.y = 0.0;
            this.render();
            if (this.onUpdate) this.onUpdate(0.0, 0.0, false);
        }

        render() {
            const ctx = this.ctx;
            const c = this.center;
            ctx.clearRect(0, 0, this.size, this.size);

            // 1. Base concentric circles
            ctx.beginPath();
            ctx.arc(c, c, this.maxRadius, 0, Math.PI * 2);
            ctx.strokeStyle = this.active ? 'rgba(0, 240, 255, 0.4)' : 'rgba(255, 255, 255, 0.12)';
            ctx.lineWidth = 2;
            ctx.stroke();

            // 2. Dial ticks
            for (let i = 0; i < 8; i++) {
                const a = (i * Math.PI) / 4;
                const r1 = this.maxRadius - 6;
                const r2 = this.maxRadius + 2;
                ctx.beginPath();
                ctx.moveTo(c + Math.cos(a) * r1, c + Math.sin(a) * r1);
                ctx.lineTo(c + Math.cos(a) * r2, c + Math.sin(a) * r2);
                ctx.strokeStyle = 'rgba(0, 240, 255, 0.3)';
                ctx.stroke();
            }

            // 3. Joystick knob position
            const knobX = c + this.x * this.maxRadius;
            const knobY = c - this.y * this.maxRadius;

            // Connecting laser line
            if (this.active) {
                ctx.beginPath();
                ctx.moveTo(c, c);
                ctx.lineTo(knobX, knobY);
                ctx.strokeStyle = 'rgba(0, 255, 157, 0.6)';
                ctx.lineWidth = 3;
                ctx.stroke();
            }

            // Joystick knob (Glow Knob)
            ctx.beginPath();
            ctx.arc(knobX, knobY, this.knobRadius, 0, Math.PI * 2);
            const grad = ctx.createRadialGradient(knobX, knobY, 2, knobX, knobY, this.knobRadius);
            if (this.active) {
                grad.addColorStop(0, '#00ff9d');
                grad.addColorStop(1, '#008f58');
            } else {
                grad.addColorStop(0, '#38bdf8');
                grad.addColorStop(1, '#0369a1');
            }
            ctx.fillStyle = grad;
            ctx.shadowColor = this.active ? '#00ff9d' : '#38bdf8';
            ctx.shadowBlur = this.active ? 15 : 6;
            ctx.fill();
            ctx.shadowBlur = 0;

            // Knob center point
            ctx.beginPath();
            ctx.arc(knobX, knobY, 4, 0, Math.PI * 2);
            ctx.fillStyle = '#ffffff';
            ctx.fill();
        }
    }

    // --- Instantiate Dual Joysticks ---
    const joyLeft = new VirtualJoystick(dom.leftCanvas, (x, y, active) => {
        state.leftJoy.x = x;
        state.leftJoy.y = y;
        state.leftJoy.active = active;

        // Compute velocity display (Forward vx, Lateral vy)
        const vx = (y * 0.35 * state.speedGain).toFixed(2);
        const vy = (x * 0.20 * state.speedGain).toFixed(2);
        dom.leftReadout.textContent = `Vx: ${vx} | Vy: ${state.crabMode ? vy : '0.00'}`;
        checkMotionState();
    });

    const joyRight = new VirtualJoystick(dom.rightCanvas, (x, y, active) => {
        state.rightJoy.x = x;
        state.rightJoy.y = y;
        state.rightJoy.active = active;

        // Yaw rotation angular velocity
        const yaw = (-x * 0.65 * state.speedGain).toFixed(2);
        dom.rightReadout.textContent = `Yaw: ${yaw} rad/s`;
        checkMotionState();
    });

    // --- Deadman Switch Logic & Control Transmission Loop (25Hz) ---
    function checkMotionState() {
        const lateralActive = state.crabMode && Math.abs(state.leftJoy.x) > 0.05;
        const isMoving = lateralActive ||
                         Math.abs(state.leftJoy.y) > 0.05 ||
                         Math.abs(state.rightJoy.x) > 0.05;

        state.isMoving = isMoving;

        if (isMoving) {
            dom.deadmanBadge.className = 'deadman-badge driving';
            dom.deadmanText.textContent = 'DRIVING (MOVING)';
            state.lastSentStop = false;
        } else {
            dom.deadmanBadge.className = 'deadman-badge brake';
            dom.deadmanText.textContent = 'BRAKE (STOP)';
        }
    }

    function controlLoop() {
        if (state.isMoving) {
            // In motion: Send real-time velocity command
            // Per Chica protocol: walk:turn,forward,anim or walkonnx:turn,forward,anim
            const forward = state.leftJoy.y * state.speedGain;
            const strafe = state.crabMode ? (state.leftJoy.x * state.speedGain) : 0.0;
            const turn = -state.rightJoy.x * state.speedGain;

            const cmdPayload = {
                type: 'walk',
                mode: state.mode, // 'onnx' or 'tripod'
                forward: parseFloat(forward.toFixed(3)),
                strafe: parseFloat(strafe.toFixed(3)),
                turn: parseFloat(turn.toFixed(3)),
                crab: state.crabMode
            };

            sendWs(cmdPayload);
            dom.cmdStream.textContent = `WALK [F:${cmdPayload.forward} T:${cmdPayload.turn}${state.crabMode ? ' S:' + cmdPayload.strafe : ''}]`;
        } else if (!state.lastSentStop) {
            // Deadman Switch triggered (hands released): Send brake stop command
            sendWs({ type: 'stop' });
            dom.cmdStream.textContent = 'STOP (WALKCLEAR)';
            state.lastSentStop = true;
        }
    }

    // Run control loop at 25Hz (40ms)
    setInterval(controlLoop, 40);

    // --- UI Button Event Bindings ---
    // Gait mode toggle
    dom.btnModeOnnx.addEventListener('click', () => {
        state.mode = 'onnx';
        dom.btnModeOnnx.classList.add('active');
        dom.btnModeTripod.classList.remove('active');
        sendWs({ type: 'cmd', command: 'onnx on' });
    });

    dom.btnModeTripod.addEventListener('click', () => {
        state.mode = 'tripod';
        dom.btnModeTripod.classList.add('active');
        dom.btnModeOnnx.classList.remove('active');
        sendWs({ type: 'cmd', command: 'onnx off' });
    });

    // Relay / Torque power
    dom.btnTorque.addEventListener('click', () => {
        sendWs({ type: 'cmd', command: 'torque' });
    });

    // Stand / Sit posture
    dom.btnStand.addEventListener('click', () => {
        sendWs({ type: 'cmd', command: 'sit' });
    });

    // Emergency Stop (E-Stop)
    dom.btnEstop.addEventListener('click', () => {
        // Send stop and estop commands
        sendWs({ type: 'stop' });
        sendWs({ type: 'estop' });
        sendWs({ type: 'cmd', command: 'estop' });
        // Only toggle torque if relay was active to prevent turning relay on
        if (state.isTorqueOn) {
            sendWs({ type: 'cmd', command: 'torque' });
        }
        joyLeft.reset();
        joyRight.reset();
        state.isMoving = false;
        dom.cmdStream.textContent = 'E-STOP (HALT)';
        dom.deadmanBadge.className = 'deadman-badge brake';
        dom.deadmanText.textContent = 'E-STOP HALT';
    });

    // Crab mode
    dom.btnCrab.addEventListener('click', () => {
        state.crabMode = !state.crabMode;
        dom.btnCrab.classList.toggle('active', state.crabMode);
        dom.crabText.textContent = state.crabMode ? 'ON' : 'OFF';
        sendWs({ type: 'cmd', command: 'crab' });
    });

    // High clearance mode
    dom.btnClearance.addEventListener('click', () => {
        state.highClearance = !state.highClearance;
        dom.btnClearance.classList.toggle('active', state.highClearance);
        dom.clearanceText.textContent = state.highClearance ? 'ON' : 'OFF';
        sendWs({ type: 'cmd', command: state.highClearance ? 'clearance on' : 'clearance off' });
    });

    // Ground contact calibration
    dom.btnCalibrate.addEventListener('click', () => {
        if (confirm('Start hexapod ground contact sensor calibration routine (Calibrate)?')) {
            sendWs({ type: 'cmd', command: 'calibrate' });
        }
    });

    // Speed multiplier slider
    dom.speedSlider.addEventListener('input', (e) => {
        state.speedGain = parseFloat(e.target.value);
        dom.speedVal.textContent = `${state.speedGain.toFixed(1)}x`;
    });

    // Initialize WebSocket connection
    initWebSocket();
})();
