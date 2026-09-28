#!/usr/bin/env python3
"""Verify Android Servo2040 handling against the firmware-derived emulator."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from servo2040_protocol_emulator import (
    CURR,
    GET_CMD,
    RELAY,
    SET_CMD,
    TS1,
    VOLT,
    Servo2040ProtocolEmulator,
    encode14,
)


ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build" / "servo2040_protocol"
SRC = BUILD / "src"
CLASSES = BUILD / "classes"


USB_SERIAL_PORT_STUB = r"""
package com.hoho.android.usbserial.driver;

import java.io.IOException;

public interface UsbSerialPort {
    int PARITY_NONE = 0;
    void write(byte[] data, int timeout) throws IOException;
    int read(byte[] dest, int timeout) throws IOException;
    default void open(android.hardware.usb.UsbDeviceConnection connection) throws IOException { }
    default void close() throws IOException { }
    default void setParameters(int baudRate, int dataBits, int stopBits, int parity) throws IOException { }
    default void setDTR(boolean value) throws IOException { }
}
"""

USB_DEVICE_CONNECTION_STUB = r"""
package android.hardware.usb;

public final class UsbDeviceConnection {
}
"""


HARNESS = r"""
package com.makeyourpet.chicaserver.hardware;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import java.io.IOException;
import java.util.Arrays;

public final class Servo2040ProtocolHarness {
    private static final int SET_CMD = 0xd3;
    private static final int GET_CMD = 0xc7;
    private static final int SERVO18 = 17;
    private static final int TS1 = 18;
    private static final int TS6 = 23;
    private static final int CURR = 24;
    private static final int VOLT = 25;
    private static final int RELAY = 26;
    private static final double RATIO = 310.3d;
    private static final double CURRENT_LSB = 0.0814d;

    public static void main(String[] args) {
        FakeServo2040Port port = new FakeServo2040Port();
        ChicaConfigPins pins = ChicaConfigPins.parse(
                "RELAY P26\n"
                        + "TS_L1 P18\nTS_L2 P19\nTS_L3 P20\n"
                        + "TS_R1 P21\nTS_R2 P22\nTS_R3 P23\n"
                        + "CUR P24\nVOL P25\n");
        UsbSerialServoBackend backend = new UsbSerialServoBackend(
                UsbSerialServoBackend.BoardProtocol.SERVO2040, port, null, pins, null);

        int[] pulses = pulses();
        backend.setServoPulses(pulses);
        check(Arrays.equals(port.servos, pulses), "pulse SET updates all 18 servos");
        check(!port.relayEnabled, "pulse SET does not enable relay");

        backend.setRelay(true);
        check(port.relayEnabled && Boolean.TRUE.equals(port.outputs.get(RELAY)), "relay on");
        backend.setRelay(false);
        check(!port.relayEnabled && Boolean.FALSE.equals(port.outputs.get(RELAY)), "relay off");

        port.setAnalog(new double[] {0.0d, 3.3d, 1.65d, 0.33d, 2.2d, 2.75d}, 7.4d, 2.6d);
        backend.refreshTelemetry();
        assertBytes(port.lastWrite, new int[] {
                0xc7, 18, 1, 0xc7, 19, 1, 0xc7, 20, 1, 0xc7, 21, 1,
                0xc7, 22, 1, 0xc7, 23, 1, 0xc7, 25, 1, 0xc7, 24, 1
        }, "poll frame order");
        assertClose(backend.voltage(), Math.round(7.4d * RATIO) / 310.29998779296875d,
                "voltage conversion");
        assertClose(backend.current(), (Math.round(2.6d / CURRENT_LSB) + 512 - 512)
                * 0.08139999955892563d, "current conversion");
        assertArrayClose(backend.legTouches(), new double[] {
                Math.round(0.0d * RATIO) / 1024.0d,
                Math.round(3.3d * RATIO) / 1024.0d,
                Math.round(1.65d * RATIO) / 1024.0d,
                Math.round(0.33d * RATIO) / 1024.0d,
                Math.round(2.2d * RATIO) / 1024.0d,
                Math.round(2.75d * RATIO) / 1024.0d,
        }, "touch conversion");

        byte[] reply = port.exchange(new byte[] {(byte) GET_CMD, 0, 3});
        assertBytes(reply, new int[] {
                0xc7, 0, 3,
                pulses[0] & 0x7f, (pulses[0] >> 7) & 0x7f,
                pulses[1] & 0x7f, (pulses[1] >> 7) & 0x7f,
                pulses[2] & 0x7f, (pulses[2] >> 7) & 0x7f,
        }, "multi-get echo and values");

        System.out.println("servo2040 java backend exact=true");
    }

    private static int[] pulses() {
        int[] out = new int[18];
        for (int i = 0; i < out.length; i++) out[i] = 810 + (i * 41);
        return out;
    }

    private static void assertArrayClose(double[] actual, double[] expected, String label) {
        check(actual.length == expected.length, label + " length");
        for (int i = 0; i < actual.length; i++) assertClose(actual[i], expected[i], label + " " + i);
    }

    private static void assertClose(double actual, double expected, String label) {
        if (Double.isNaN(actual) || Math.abs(actual - expected) > 0.000000001d) {
            throw new AssertionError(label + " expected=" + expected + " actual=" + actual);
        }
    }

    private static void assertBytes(byte[] actual, int[] expected, String label) {
        check(actual.length == expected.length, label + " length expected=" + expected.length
                + " actual=" + actual.length);
        for (int i = 0; i < expected.length; i++) {
            check((actual[i] & 0xff) == expected[i], label + " byte " + i);
        }
    }

    private static void check(boolean condition, String label) {
        if (!condition) throw new AssertionError(label);
    }

    private static final class FakeServo2040Port implements UsbSerialPort {
        private final int[] servos = filled(1500);
        private final boolean[] digital = new boolean[29];
        private final java.util.Map<Integer, Boolean> outputs = new java.util.HashMap<>();
        private final double[] touches = new double[6];
        private boolean relayEnabled;
        private double voltage;
        private double current;
        private byte[] pendingReply = new byte[0];
        private byte[] lastWrite = new byte[0];

        void setAnalog(double[] touchVoltages, double voltage, double current) {
            System.arraycopy(touchVoltages, 0, touches, 0, touches.length);
            this.voltage = voltage;
            this.current = current;
        }

        @Override
        public void write(byte[] data, int timeout) {
            lastWrite = data.clone();
            pendingReply = exchange(data);
        }

        @Override
        public int read(byte[] dest, int timeout) {
            if (pendingReply.length == 0) return 0;
            int count = Math.min(dest.length, pendingReply.length);
            System.arraycopy(pendingReply, 0, dest, 0, count);
            pendingReply = Arrays.copyOfRange(pendingReply, count, pendingReply.length);
            return count;
        }

        byte[] exchange(byte[] data) {
            java.io.ByteArrayOutputStream reply = new java.io.ByteArrayOutputStream();
            int offset = 0;
            while (offset < data.length) {
                int cmd = data[offset++] & 0xff;
                if ((cmd & 0x80) == 0) continue;
                if (offset + 2 > data.length) break;
                int start = data[offset++] & 0xff;
                int count = data[offset++] & 0xff;
                if (cmd == SET_CMD) {
                    for (int i = 0; i < count && offset + 1 < data.length; i++) {
                        int value = (data[offset++] & 0x7f) | ((data[offset++] & 0x7f) << 7);
                        int pin = start + i;
                        if (pin <= SERVO18) {
                            servos[pin] = value;
                        } else if (pin >= RELAY) {
                            boolean enabled = value != 0;
                            digital[pin] = enabled;
                            outputs.put(pin, enabled);
                            if (pin == RELAY) relayEnabled = enabled;
                        }
                    }
                } else if (cmd == GET_CMD) {
                    reply.write(GET_CMD);
                    reply.write(start);
                    reply.write(count);
                    for (int pin = start; pin < start + count; pin++) write14(reply, readRaw(pin));
                } else {
                    break;
                }
            }
            return reply.toByteArray();
        }

        private int readRaw(int pin) {
            if (pin <= SERVO18) return servos[pin];
            if (pin >= TS1 && pin <= TS6) return (int) Math.round(touches[pin - TS1] * RATIO);
            if (pin == CURR) return (int) Math.round(current / CURRENT_LSB) + 512;
            if (pin == VOLT) return (int) Math.round(voltage * RATIO);
            return 0;
        }

        private static void write14(java.io.ByteArrayOutputStream out, int value) {
            out.write(value & 0x7f);
            out.write((value >> 7) & 0x7f);
        }

        private static int[] filled(int value) {
            int[] out = new int[18];
            Arrays.fill(out, value);
            return out;
        }
    }
}
"""


def javac() -> str:
    java_home = os.environ.get("JAVA_HOME")
    if java_home:
        candidate = Path(java_home) / "bin" / "javac"
        if candidate.exists():
            return str(candidate)
        candidate_exe = Path(java_home) / "bin" / "javac.exe"
        if candidate_exe.exists():
            return str(candidate_exe)
    found = shutil.which("javac")
    return found if found else "javac"


def java() -> str:
    java_home = os.environ.get("JAVA_HOME")
    if java_home:
        candidate = Path(java_home) / "bin" / "java"
        if candidate.exists():
            return str(candidate)
        candidate_exe = Path(java_home) / "bin" / "java.exe"
        if candidate_exe.exists():
            return str(candidate_exe)
    found = shutil.which("java")
    return found if found else "java"


def verify_python_emulator() -> None:
    emu = Servo2040ProtocolEmulator()
    pulses = [810 + (i * 41) for i in range(18)]
    frame = bytearray((SET_CMD, 0, 18))
    for pulse in pulses:
        frame.extend(encode14(pulse))
    assert emu.exchange(frame) == b""
    assert emu.servos == pulses
    assert not emu.relay_enabled

    assert emu.exchange(bytes((SET_CMD, RELAY, 1, 1, 0))) == b""
    assert emu.relay_enabled
    assert emu.outputs[RELAY] is True
    assert emu.exchange(bytes((SET_CMD, RELAY, 1, 0, 0))) == b""
    assert not emu.relay_enabled

    emu.set_analog(touches=[0.0, 3.3, 1.65, 0.33, 2.2, 2.75], voltage=7.4, current_amps=2.6)
    request = bytes((GET_CMD, TS1, 6, GET_CMD, VOLT, 1, GET_CMD, CURR, 1))
    reply = emu.exchange(request)
    expected = bytearray((GET_CMD, TS1, 6))
    for touch in emu.touch_voltages:
        expected.extend(encode14(round(touch * 310.3)))
    expected.extend((GET_CMD, VOLT, 1))
    expected.extend(encode14(round(7.4 * 310.3)))
    expected.extend((GET_CMD, CURR, 1))
    expected.extend(encode14(round(2.6 / 0.0814) + 512))
    assert reply == bytes(expected)
    print("servo2040 python emulator exact=true")


def main() -> int:
    verify_python_emulator()
    shutil.rmtree(BUILD, ignore_errors=True)
    (SRC / "com" / "hoho" / "android" / "usbserial" / "driver").mkdir(parents=True)
    (SRC / "com" / "makeyourpet" / "chicaserver" / "hardware").mkdir(parents=True)
    (SRC / "android" / "hardware" / "usb").mkdir(parents=True)
    CLASSES.mkdir(parents=True)
    (SRC / "com" / "hoho" / "android" / "usbserial" / "driver" / "UsbSerialPort.java").write_text(
            USB_SERIAL_PORT_STUB, encoding="utf-8")
    (SRC / "android" / "hardware" / "usb" / "UsbDeviceConnection.java").write_text(
            USB_DEVICE_CONNECTION_STUB, encoding="utf-8")
    (SRC / "com" / "makeyourpet" / "chicaserver" / "hardware" / "Servo2040ProtocolHarness.java").write_text(
            HARNESS, encoding="utf-8")

    sources = [
        SRC / "com" / "hoho" / "android" / "usbserial" / "driver" / "UsbSerialPort.java",
        SRC / "android" / "hardware" / "usb" / "UsbDeviceConnection.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/ServoBackend.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/ServoPacketEncoder.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/ChicaConfigPins.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/UsbSerialServoBackend.java",
        SRC / "com" / "makeyourpet" / "chicaserver" / "hardware" / "Servo2040ProtocolHarness.java",
    ]
    compiler = javac()
    runner = java()
    if not (Path(compiler).is_file() or shutil.which(compiler)):
        print(f"javac not available ({compiler}), skipping Java harness verification")
        return 0
    if not (Path(runner).is_file() or shutil.which(runner)):
        print(f"java not available ({runner}), skipping Java harness verification")
        return 0

    subprocess.run([compiler, "-d", str(CLASSES), *map(str, sources)], check=True, cwd=ROOT)
    subprocess.run([runner, "-cp", str(CLASSES),
                    "com.makeyourpet.chicaserver.hardware.Servo2040ProtocolHarness"],
                   check=True, cwd=ROOT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
