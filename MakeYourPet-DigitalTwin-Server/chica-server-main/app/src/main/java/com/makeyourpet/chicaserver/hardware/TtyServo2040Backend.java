package com.makeyourpet.chicaserver.hardware;

import android.util.Log;
import java.io.File;
import java.util.Arrays;

final class TtyServo2040Backend implements ServoBackend {
    private static final String TAG = "CHICA_TTY";
    private static final int TIMEOUT_MS = 500;
    private static final int MAX_DEBUG_LOGS = 24;

    private final String path;
    private final ChicaConfigPins pins;
    private int fd;
    private volatile boolean connected;
    private volatile boolean relayEnabled;
    private volatile double voltage = Double.NaN;
    private volatile double current = Double.NaN;
    private byte[] stagedServoFrame;
    private final double[] legTouches = {
            Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN
    };
    private final int[] touchRaw = minRawArray(6);
    private int curRaw = Integer.MIN_VALUE;
    private int volRaw = Integer.MIN_VALUE;
    private byte[] pollFrame;
    private int debugLogCount;

    private TtyServo2040Backend(String path, ChicaConfigPins pins, int fd) {
        this.path = path;
        this.pins = pins;
        this.fd = fd;
        this.connected = true;
    }

    static TtyServo2040Backend openIfAvailable(String path, ChicaConfigPins pins) {
        File file = new File(path);
        if (!file.exists() || !file.canRead() || !file.canWrite()) return null;
        int fd = NativeTty.nativeOpenRaw(path);
        if (fd < 0) return null;
        return new TtyServo2040Backend(path, pins, fd);
    }

    @Override
    public String name() {
        return "servo2040-tty:" + path;
    }

    @Override
    public boolean isConnected() {
        return connected;
    }

    @Override
    public synchronized void stageServoPulses(int[] pulses) {
        if (stagedServoFrame == null) stagedServoFrame = new byte[39];
        ServoPacketEncoder.servo2040Pulses(pulses, stagedServoFrame);
    }

    @Override
    public synchronized void flushServoPulses() {
        if (stagedServoFrame != null) write(stagedServoFrame);
    }

    @Override
    public synchronized void setRelay(boolean enabled) {
        relayEnabled = enabled;
        ChicaConfigPins.UsbPin relay = pins.relayPin;
        if (relay == null) return;
        write(ServoPacketEncoder.servo2040DigitalOut(relay.pin, enabled));
    }

    @Override
    public synchronized void refreshTelemetry() {
        int count = countConfiguredPins();
        if (count == 0) return;
        int requestLength = count * 3;
        if (pollFrame == null || pollFrame.length != requestLength) {
            pollFrame = buildPollFrame(requestLength);
        }
        byte[] reply = new byte[count * 5];
        if (!writeAndReadExact(pollFrame, reply)) {
            debugLog("telemetry read failed request=" + hex(pollFrame));
            clearRawTelemetry();
            voltage = Double.NaN;
            current = Double.NaN;
            Arrays.fill(legTouches, Double.NaN);
            return;
        }
        int index = 0;
        for (int leg = 0; leg < pins.touchPins.length; leg++) {
            ChicaConfigPins.UsbPin pin = pins.touchPins[leg];
            if (pin != null && pin.primaryBoard) {
                touchRaw[leg] = rawServo2040(reply, index);
                index++;
            }
        }
        if (isOnPrimary(pins.volPin)) {
            volRaw = rawServo2040(reply, index);
            index++;
        }
        if (isOnPrimary(pins.curPin)) {
            curRaw = rawServo2040(reply, index);
        }
        voltage = volRaw == Integer.MIN_VALUE ? Double.NaN : volRaw / 310.29998779296875d;
        current = curRaw == Integer.MIN_VALUE ? Double.NaN
                : (curRaw - 512.0d) * 0.08139999955892563d;
        for (int i = 0; i < legTouches.length; i++) {
            int raw = touchRaw[i];
            legTouches[i] = raw == Integer.MIN_VALUE ? Double.NaN : raw / 1024.0d;
        }
        debugLog("telemetry ok request=" + hex(pollFrame)
                + " reply=" + hex(reply)
                + " volRaw=" + volRaw
                + " curRaw=" + curRaw
                + " voltage=" + voltage
                + " current=" + current);
    }

    @Override
    public double voltage() {
        return voltage;
    }

    @Override
    public double current() {
        return current;
    }

    @Override
    public synchronized double[] legTouches() {
        return legTouches.clone();
    }

    public boolean relayEnabled() {
        return relayEnabled;
    }

    private boolean writeAndReadExact(byte[] request, byte[] reply) {
        if (!write(request)) return false;
        long deadline = System.currentTimeMillis() + TIMEOUT_MS;
        int received = 0;
        byte[] buffer = new byte[256];
        while (received < reply.length && System.currentTimeMillis() < deadline) {
            int read = read(buffer);
            if (read > 0) {
                int copy = Math.min(read, reply.length - received);
                System.arraycopy(buffer, 0, reply, received, copy);
                received += copy;
            } else {
                sleep(5L);
            }
        }
        connected = received == reply.length;
        if (!connected) {
            debugLog("short read received=" + received
                    + " expected=" + reply.length
                    + " partial=" + hex(reply, received));
        }
        return connected;
    }

    private boolean write(byte[] frame) {
        if (fd < 0) {
            connected = false;
            return false;
        }
        int offset = 0;
        long deadline = System.currentTimeMillis() + TIMEOUT_MS;
        while (offset < frame.length && System.currentTimeMillis() < deadline) {
            int written = NativeTty.nativeWrite(fd, frame, offset, frame.length - offset);
            if (written > 0) {
                offset += written;
            } else if (written == 0 || written == -11 || written == -35) {
                sleep(5L);
            } else {
                connected = false;
                return false;
            }
        }
        connected = offset == frame.length;
        if (!connected) {
            debugLog("short write written=" + offset + " expected=" + frame.length);
        }
        return connected;
    }

    private int read(byte[] buffer) {
        int read = NativeTty.nativeRead(fd, buffer, 0, buffer.length);
        if (read >= 0) return read;
        if (read == -11 || read == -35) return 0;
        debugLog("read errno=" + read);
        connected = false;
        return -1;
    }

    @Override
    public synchronized void close() {
        connected = false;
        if (fd >= 0) {
            NativeTty.nativeClose(fd);
            fd = -1;
        }
    }

    @Override
    protected void finalize() throws Throwable {
        try {
            close();
        } finally {
            super.finalize();
        }
    }

    private byte[] buildPollFrame(int length) {
        byte[] frame = new byte[length];
        int offset = 0;
        for (ChicaConfigPins.UsbPin pin : pins.touchPins) {
            if (pin != null && pin.primaryBoard) offset = appendRead(frame, offset, pin.pin);
        }
        if (isOnPrimary(pins.volPin)) offset = appendRead(frame, offset, pins.volPin.pin.pin);
        if (isOnPrimary(pins.curPin)) appendRead(frame, offset, pins.curPin.pin.pin);
        return frame;
    }

    private int appendRead(byte[] frame, int offset, int pin) {
        frame[offset++] = (byte) 0xc7;
        frame[offset++] = (byte) pin;
        frame[offset++] = 1;
        return offset;
    }

    private int countConfiguredPins() {
        int count = 0;
        for (ChicaConfigPins.UsbPin pin : pins.touchPins) {
            if (pin != null && pin.primaryBoard) count++;
        }
        if (isOnPrimary(pins.volPin)) count++;
        if (isOnPrimary(pins.curPin)) count++;
        return count;
    }

    private static boolean isOnPrimary(ChicaConfigPins.AnalogPin pin) {
        return pin != null && pin.pin != null && pin.pin.primaryBoard;
    }

    private static int rawServo2040(byte[] reply, int index) {
        int offset = index * 5;
        if (offset + 4 >= reply.length) return Integer.MIN_VALUE;
        return (reply[offset + 3] & 0x7f) | ((reply[offset + 4] & 0x7f) << 7);
    }

    private void clearRawTelemetry() {
        Arrays.fill(touchRaw, Integer.MIN_VALUE);
        curRaw = Integer.MIN_VALUE;
        volRaw = Integer.MIN_VALUE;
    }

    private static int[] minRawArray(int length) {
        int[] values = new int[length];
        Arrays.fill(values, Integer.MIN_VALUE);
        return values;
    }

    private static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException error) {
            Thread.currentThread().interrupt();
        }
    }

    private void debugLog(String message) {
        if (debugLogCount < MAX_DEBUG_LOGS) {
            debugLogCount++;
            Log.i(TAG, message);
        }
    }

    private static String hex(byte[] data) {
        return hex(data, data.length);
    }

    private static String hex(byte[] data, int length) {
        StringBuilder text = new StringBuilder(length * 2);
        for (int i = 0; i < length && i < data.length; i++) {
            int value = data[i] & 0xff;
            if (value < 16) text.append('0');
            text.append(Integer.toHexString(value));
        }
        return text.toString();
    }
}
