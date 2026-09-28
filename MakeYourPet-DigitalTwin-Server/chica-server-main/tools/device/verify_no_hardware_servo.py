#!/usr/bin/env python3
"""No-hardware servo checks for packet encoding and virtual telemetry."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build" / "no_hardware_servo"
SRC = BUILD / "src"
CLASSES = BUILD / "classes"


HARNESS = r"""
import com.makeyourpet.chicaserver.control.SurfaceStatus;
import com.makeyourpet.chicaserver.hardware.ServoPacketEncoder;
import com.makeyourpet.chicaserver.hardware.VirtualServoBackend;
import java.util.Arrays;

public final class NoHardwareServoHarness {
    private static final int[] FEMUR_PINS = {16, 10, 4, 13, 7, 1};
    private static final int[] THRESHOLDS = {8, 10, 12, 8, 10, 12};

    public static void main(String[] args) {
        verifyServo2040Pulses();
        verifyPololuPulses();
        verifyDigitalOut();
        verifySurfaceStatus();
        verifyVirtualBackend();
        System.out.println("no_hardware_servo exact=true");
    }

    private static void verifyServo2040Pulses() {
        int[] pulses = pulses();
        byte[] frame = ServoPacketEncoder.servo2040Pulses(pulses);
        check(frame.length == 39, "servo2040 frame length");
        check(u(frame[0]) == 0xd3 && frame[1] == 0 && frame[2] == 18, "servo2040 header");
        for (int i = 0; i < pulses.length; i++) {
            int offset = (i * 2) + 3;
            check(u(frame[offset]) == (pulses[i] & 0x7f), "servo2040 low " + i);
            check(u(frame[offset + 1]) == ((pulses[i] >> 7) & 0x7f), "servo2040 high " + i);
        }
    }

    private static void verifyPololuPulses() {
        int[] pulses = pulses();
        byte[] frame = ServoPacketEncoder.pololuPulses(pulses);
        check(frame.length == 39, "pololu frame length");
        check(u(frame[0]) == 0x9f && frame[1] == 18 && frame[2] == 0, "pololu header");
        for (int i = 0; i < pulses.length; i++) {
            int target = pulses[i] * 4;
            int offset = (i * 2) + 3;
            check(u(frame[offset]) == (target & 0x7f), "pololu low " + i);
            check(u(frame[offset + 1]) == ((target >> 7) & 0x7f), "pololu high " + i);
        }
    }

    private static void verifyDigitalOut() {
        assertBytes(ServoPacketEncoder.servo2040DigitalOut(26, true),
                new int[] {0xd3, 26, 1, 1, 0}, "servo2040 relay on");
        assertBytes(ServoPacketEncoder.servo2040DigitalOut(26, false),
                new int[] {0xd3, 26, 1, 0, 0}, "servo2040 relay off");
        assertBytes(ServoPacketEncoder.pololuDigitalOut(11, true),
                new int[] {0x84, 11, 88, 54}, "pololu relay on");
        assertBytes(ServoPacketEncoder.pololuDigitalOut(11, false),
                new int[] {0x84, 11, 8, 39}, "pololu relay off");
    }

    private static void verifySurfaceStatus() {
        SurfaceStatus status = new SurfaceStatus(7.4, 11.25, 12, "10.0.2.16",
                new double[] {1, 0, 1, 0, Double.NaN, 1},
                0, 0, 0, 0, 0, 0, 0, false, true, 0, 0, 0);
        check("BPS= 12|V= 7.400|I= 11.250|IP=10.0.2.16|LEGS=x-x--x"
                .equals(status.toOriginalPayload()), "surface payload");
    }

    private static void verifyVirtualBackend() {
        VirtualServoBackend backend = new VirtualServoBackend();
        check(Double.isNaN(backend.voltage()), "initial virtual voltage NaN");
        check(Double.isNaN(backend.current()), "initial virtual current NaN");
        backend.setVirtualTouchFixture(true);
        check(backend.voltage() == 7.4 && backend.current() == 7.4, "default virtual telemetry");
        check(Arrays.equals(backend.legTouches(), new double[] {0, 0, 0, 0, 0, 0}),
                "initial virtual touches");

        backend.setVirtualTelemetry(5.5, 11.2);
        check(backend.voltage() == 5.5 && backend.current() == 11.2, "scripted virtual telemetry");

        int[] raise = filled(1500);
        for (int leg = 0; leg < FEMUR_PINS.length; leg++) {
            raise[FEMUR_PINS[leg]] = leg < 3 ? 1480 : 1520;
        }
        backend.setServoPulses(raise);
        backend.refreshTelemetry();
        check(Arrays.equals(backend.legTouches(), new double[] {0, 0, 0, 0, 0, 0}),
                "touches open after raise");

        int[] contact = raise.clone();
        for (int leg = 0; leg < FEMUR_PINS.length; leg++) {
            contact[FEMUR_PINS[leg]] += leg < 3 ? THRESHOLDS[leg] : -THRESHOLDS[leg];
        }
        backend.setServoPulses(contact);
        backend.refreshTelemetry();
        check(Arrays.equals(backend.legTouches(), new double[] {1, 1, 1, 1, 1, 1}),
                "touches closed at thresholds");

        backend.setVirtualTouchFixture(false);
        check(Double.isNaN(backend.voltage()) && Double.isNaN(backend.current()),
                "fixture disabled telemetry");
    }

    private static int[] pulses() {
        int[] out = new int[18];
        for (int i = 0; i < out.length; i++) out[i] = 900 + (i * 73);
        return out;
    }

    private static int[] filled(int value) {
        int[] out = new int[18];
        Arrays.fill(out, value);
        return out;
    }

    private static void assertBytes(byte[] actual, int[] expected, String label) {
        check(actual.length == expected.length, label + " length");
        for (int i = 0; i < expected.length; i++) {
            check(u(actual[i]) == expected[i], label + " byte " + i);
        }
    }

    private static int u(byte value) {
        return value & 0xff;
    }

    private static void check(boolean condition, String label) {
        if (!condition) throw new AssertionError(label);
    }
}
"""


LOG_STUB = r"""
package android.util;

public final class Log {
    private Log() {
    }

    public static int i(String tag, String message) {
        return 0;
    }

    public static int w(String tag, String message) {
        return 0;
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


def main() -> int:
    shutil.rmtree(BUILD, ignore_errors=True)
    (SRC / "android" / "util").mkdir(parents=True)
    CLASSES.mkdir(parents=True)
    (SRC / "android" / "util" / "Log.java").write_text(LOG_STUB, encoding="utf-8")
    (SRC / "NoHardwareServoHarness.java").write_text(HARNESS, encoding="utf-8")

    sources = [
        SRC / "android" / "util" / "Log.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/ServoBackend.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/ServoPacketEncoder.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/VirtualServoBackend.java",
        ROOT / "app/src/main/java/com/makeyourpet/chicaserver/control/SurfaceStatus.java",
        SRC / "NoHardwareServoHarness.java",
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
    subprocess.run([runner, "-cp", str(CLASSES), "NoHardwareServoHarness"], check=True, cwd=ROOT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
