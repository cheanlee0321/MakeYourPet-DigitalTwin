#!/usr/bin/env python3
"""Verify the Android Pololu (Maestro) read/telemetry path with no hardware.

Mirrors verify_servo2040_protocol.py for the Pololu protocol. Drives the real
UsbSerialServoBackend in POLOLU mode through a fake serial port and checks:

- pulse output frame (0x9f) on the primary port,
- inverted relay digital-out (0x84) on the secondary board's port,
- the secondary analog poll frame (0x90 per pin, in touch->VOL->CUR order),
- the original 2-byte read decode (raw = high*255 + low) and the
  voltage/current/touch conversions from the CUR/VOL calibration.

The Pololu board cannot be tested against real hardware, so this headless
round-trip is the only verification of that path.
"""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build" / "pololu_protocol"
SRC = BUILD / "src"
CLASSES = BUILD / "classes"

# Pololu config: servos on the primary board (P..), all analog + relay on the
# secondary board (S..), matching assets/config-pololu.txt.
CONFIG = (
    "RELAY S11\n"
    "TS_L1 S5 1\nTS_L2 S3 1\nTS_L3 S1 1\n"
    "TS_R1 S4 1\nTS_R2 S2 1\nTS_R3 S0 1\n"
    "CUR S6 2.5 2.556\nVOL S7 0.0 0.2\n"
)

# Raw analog values the fake device returns (decoded as high*255+low).
TOUCH_RAW = [0, 100, 200, 300, 400, 500]   # in touch-pin (leg) order
VOL_RAW = 303
CUR_RAW = 600

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
import java.util.Arrays;

public final class PololuProtocolHarness {
    // Raw values the fake device returns, in the backend's parse order.
    static final int[] TOUCH_RAW = {0, 100, 200, 300, 400, 500};
    static final int VOL_RAW = 303;
    static final int CUR_RAW = 600;

    public static void main(String[] args) {
        FakePololuPort primary = new FakePololuPort();   // servo pulses
        FakePololuPort secondary = new FakePololuPort();  // relay + analog reads
        ChicaConfigPins pins = ChicaConfigPins.parse(CONFIG);
        UsbSerialServoBackend backend = new UsbSerialServoBackend(
                UsbSerialServoBackend.BoardProtocol.POLOLU, primary, secondary, pins, null);

        // --- pulse output goes to the primary port as a 0x9f frame ---
        int[] pulses = new int[18];
        for (int i = 0; i < 18; i++) pulses[i] = 810 + (i * 41);
        backend.setServoPulses(pulses);
        check(primary.lastWrite.length == 39, "pulse frame length");
        check((primary.lastWrite[0] & 0xff) == 0x9f, "pulse frame header 0x9f");

        // --- relay (S11) goes to the secondary port, original inverts state ---
        backend.setRelay(true);
        assertBytes(secondary.lastWrite, new int[] {0x84, 11, 8, 39}, "relay on (inverted)");
        backend.setRelay(false);
        assertBytes(secondary.lastWrite, new int[] {0x84, 11, 88, 54}, "relay off (inverted)");

        // --- analog poll + parse + conversion on the secondary board ---
        secondary.setAnalog(TOUCH_RAW, VOL_RAW, CUR_RAW);
        backend.refreshTelemetry();

        // poll frame: touch pins (TS_L1..TS_R3 -> S5,S3,S1,S4,S2,S0), then VOL(7), CUR(6)
        assertBytes(secondary.lastWrite, new int[] {
                0x90, 5, 0x90, 3, 0x90, 1, 0x90, 4, 0x90, 2, 0x90, 0,
                0x90, 7, 0x90, 6,
        }, "poll frame order");

        // voltage = ((raw*5/1023) - zero) / (one - zero), VOL S7 0.0 0.2
        double expectedV = (((VOL_RAW * 5.0d) / 1023.0d) - 0.0d) / (0.2d - 0.0d);
        assertClose(backend.voltage(), expectedV, "voltage conversion");
        // current = ((raw*5/1023) - 2.5) / (2.556 - 2.5)
        double expectedI = (((CUR_RAW * 5.0d) / 1023.0d) - 2.5d) / (2.556d - 2.5d);
        assertClose(backend.current(), expectedI, "current conversion");
        // touch = raw / 1024
        double[] expectedTouch = new double[6];
        for (int i = 0; i < 6; i++) expectedTouch[i] = TOUCH_RAW[i] / 1024.0d;
        assertArrayClose(backend.legTouches(), expectedTouch, "touch conversion");

        System.out.println("pololu java backend exact=true");
    }

    static final String CONFIG =
            "RELAY S11\n"
            + "TS_L1 S5 1\nTS_L2 S3 1\nTS_L3 S1 1\n"
            + "TS_R1 S4 1\nTS_R2 S2 1\nTS_R3 S0 1\n"
            + "CUR S6 2.5 2.556\nVOL S7 0.0 0.2\n";

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

    // Fake Pololu port: records the last write and, for a 0x90 read frame,
    // returns one 2-byte little-endian-ish value per channel encoded as the
    // backend decodes it (raw = high*255 + low).
    private static final class FakePololuPort implements UsbSerialPort {
        private final int[] touchRaw = new int[6];
        private int volRaw;
        private int curRaw;
        private byte[] lastWrite = new byte[0];
        private byte[] pendingReply = new byte[0];

        void setAnalog(int[] touch, int vol, int cur) {
            System.arraycopy(touch, 0, touchRaw, 0, 6);
            this.volRaw = vol;
            this.curRaw = cur;
        }

        @Override
        public void write(byte[] data, int timeout) {
            lastWrite = data.clone();
            pendingReply = (data.length > 0 && (data[0] & 0xff) == 0x90) ? buildReply(data) : new byte[0];
        }

        @Override
        public int read(byte[] dest, int timeout) {
            if (pendingReply.length == 0) return 0;
            int count = Math.min(dest.length, pendingReply.length);
            System.arraycopy(pendingReply, 0, dest, 0, count);
            pendingReply = Arrays.copyOfRange(pendingReply, count, pendingReply.length);
            return count;
        }

        // Reply is one raw value per [0x90, pin] pair, in request order.
        private byte[] buildReply(byte[] request) {
            byte[] reply = new byte[request.length];
            int channel = 0;
            for (int off = 0; off + 1 < request.length; off += 2) {
                int pin = request[off + 1] & 0xff;
                int raw = rawForPin(pin, channel);
                reply[channel * 2] = (byte) (raw % 255);       // low
                reply[channel * 2 + 1] = (byte) (raw / 255);   // high
                channel++;
            }
            return reply;
        }

        private int rawForPin(int pin, int channel) {
            if (pin == 7) return volRaw;   // VOL S7
            if (pin == 6) return curRaw;   // CUR S6
            return touchRaw[channel];      // touch channels in request order
        }
    }
}
"""


def tool(name: str) -> str:
    java_home = os.environ.get("JAVA_HOME")
    if java_home:
        candidate = Path(java_home) / "bin" / name
        if candidate.exists():
            return str(candidate)
        candidate_exe = Path(java_home) / "bin" / f"{name}.exe"
        if candidate_exe.exists():
            return str(candidate_exe)
    found = shutil.which(name)
    return found if found else name


def verify_python_model() -> None:
    """Independent check of the decode + conversion formulas."""
    def decode(raw):
        low, high = raw % 255, raw // 255
        return (high & 0xff) * 255 + (low & 0xff)
    for raw in (*TOUCH_RAW, VOL_RAW, CUR_RAW):
        assert decode(raw) == raw, f"decode round-trip {raw}"
    voltage = (((VOL_RAW * 5.0) / 1023.0) - 0.0) / (0.2 - 0.0)
    current = (((CUR_RAW * 5.0) / 1023.0) - 2.5) / (2.556 - 2.5)
    assert voltage > 0 and current > 0
    print("pololu python model exact=true")


def main() -> int:
    verify_python_model()
    shutil.rmtree(BUILD, ignore_errors=True)
    (SRC / "com" / "hoho" / "android" / "usbserial" / "driver").mkdir(parents=True)
    hw = SRC / "com" / "makeyourpet" / "chicaserver" / "hardware"
    hw.mkdir(parents=True)
    (SRC / "android" / "hardware" / "usb").mkdir(parents=True)
    CLASSES.mkdir(parents=True)
    (SRC / "com" / "hoho" / "android" / "usbserial" / "driver" / "UsbSerialPort.java").write_text(
            USB_SERIAL_PORT_STUB, encoding="utf-8")
    (SRC / "android" / "hardware" / "usb" / "UsbDeviceConnection.java").write_text(
            USB_DEVICE_CONNECTION_STUB, encoding="utf-8")
    (hw / "PololuProtocolHarness.java").write_text(HARNESS, encoding="utf-8")

    sources = [
        SRC / "com" / "hoho" / "android" / "usbserial" / "driver" / "UsbSerialPort.java",
        SRC / "android" / "hardware" / "usb" / "UsbDeviceConnection.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/ServoBackend.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/ServoPacketEncoder.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/ChicaConfigPins.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/UsbSerialServoBackend.java",
        hw / "PololuProtocolHarness.java",
    ]
    compiler = tool("javac")
    runner = tool("java")
    if not (Path(compiler).is_file() or shutil.which(compiler)):
        print(f"javac not available ({compiler}), skipping Java harness verification")
        return 0
    if not (Path(runner).is_file() or shutil.which(runner)):
        print(f"java not available ({runner}), skipping Java harness verification")
        return 0

    subprocess.run([compiler, "-d", str(CLASSES), *map(str, sources)], check=True, cwd=ROOT)
    subprocess.run([runner, "-cp", str(CLASSES),
                    "com.makeyourpet.chicaserver.hardware.PololuProtocolHarness"],
                   check=True, cwd=ROOT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
