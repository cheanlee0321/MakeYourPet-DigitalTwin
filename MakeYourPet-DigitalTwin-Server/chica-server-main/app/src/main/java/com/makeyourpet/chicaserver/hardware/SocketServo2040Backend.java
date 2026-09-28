package com.makeyourpet.chicaserver.hardware;

import android.util.Log;
import android.os.StrictMode;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.util.Arrays;

final class SocketServo2040Backend implements ServoBackend {
    private static final String TAG = "CHICA_TCP_HW";
    private static final int CONNECT_TIMEOUT_MS = 1000;
    private static final int TIMEOUT_MS = 500;
    private static final int MAX_DEBUG_LOGS = 24;

    private final String host;
    private final int port;
    private final ChicaConfigPins pins;
    private final Socket socket;
    private final InputStream input;
    private final OutputStream output;
    private volatile boolean connected = true;
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

    private SocketServo2040Backend(String host, int port, ChicaConfigPins pins,
            Socket socket, InputStream input, OutputStream output) {
        this.host = host;
        this.port = port;
        this.pins = pins;
        this.socket = socket;
        this.input = input;
        this.output = output;
    }

    static SocketServo2040Backend openIfAvailable(String host, int port, ChicaConfigPins pins) {
        StrictMode.ThreadPolicy oldPolicy = StrictMode.getThreadPolicy();
        try {
            StrictMode.setThreadPolicy(new StrictMode.ThreadPolicy.Builder(oldPolicy)
                    .permitNetwork()
                    .build());
            Socket socket = new Socket();
            socket.connect(new InetSocketAddress(host, port), CONNECT_TIMEOUT_MS);
            socket.setSoTimeout(TIMEOUT_MS);
            socket.setTcpNoDelay(true);
            return new SocketServo2040Backend(host, port, pins, socket,
                    socket.getInputStream(), socket.getOutputStream());
        } catch (Exception error) {
            Log.i(TAG, "open failed " + error.getClass().getSimpleName() + ": "
                    + error.getMessage());
            return null;
        } finally {
            StrictMode.setThreadPolicy(oldPolicy);
        }
    }

    @Override
    public String name() {
        return "servo2040-tcp:" + host + ":" + port;
    }

    @Override
    public boolean isConnected() {
        return connected && socket.isConnected() && !socket.isClosed();
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
        // Resync before polling: discard any bytes left buffered from a prior
        // read that timed out mid-reply. Without this, a single short read
        // permanently shifts every subsequent read (decoding GET-echo header
        // bytes as telemetry -> frozen garbage). Mirrors the original's
        // drain-on-read resilience (e4.g.t consumes surplus bytes; the factory
        // drains at startup). A no-op in steady state, where nothing is buffered
        // before a request.
        drainInput();
        if (!writeAndReadExact(pollFrame, reply)) {
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
        debugLog("telemetry ok volRaw=" + volRaw + " curRaw=" + curRaw
                + " voltage=" + voltage + " current=" + current);
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

    private void drainInput() {
        StrictMode.ThreadPolicy oldPolicy = StrictMode.getThreadPolicy();
        try {
            StrictMode.setThreadPolicy(new StrictMode.ThreadPolicy.Builder(oldPolicy)
                    .permitNetwork()
                    .build());
            byte[] scratch = new byte[256];
            int available;
            while ((available = input.available()) > 0) {
                int read = input.read(scratch, 0, Math.min(available, scratch.length));
                if (read <= 0) break;
            }
        } catch (Exception ignored) {
        } finally {
            StrictMode.setThreadPolicy(oldPolicy);
        }
    }

    private boolean writeAndReadExact(byte[] request, byte[] reply) {
        if (!write(request)) return false;
        StrictMode.ThreadPolicy oldPolicy = StrictMode.getThreadPolicy();
        int received = 0;
        try {
            StrictMode.setThreadPolicy(new StrictMode.ThreadPolicy.Builder(oldPolicy)
                    .permitNetwork()
                    .build());
            while (received < reply.length) {
                int read = input.read(reply, received, reply.length - received);
                if (read < 0) break;
                received += read;
            }
        } catch (Exception error) {
            debugLog("read failed " + error.getClass().getSimpleName());
            connected = false;
            return false;
        } finally {
            StrictMode.setThreadPolicy(oldPolicy);
        }
        connected = received == reply.length;
        if (!connected) {
            debugLog("short read received=" + received + " expected=" + reply.length);
        }
        return connected;
    }

    private boolean write(byte[] frame) {
        StrictMode.ThreadPolicy oldPolicy = StrictMode.getThreadPolicy();
        try {
            StrictMode.setThreadPolicy(new StrictMode.ThreadPolicy.Builder(oldPolicy)
                    .permitNetwork()
                    .build());
            output.write(frame);
            output.flush();
            return true;
        } catch (Exception error) {
            debugLog("write failed " + error.getClass().getSimpleName());
            connected = false;
            return false;
        } finally {
            StrictMode.setThreadPolicy(oldPolicy);
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

    private void debugLog(String message) {
        if (debugLogCount < MAX_DEBUG_LOGS) {
            debugLogCount++;
            Log.i(TAG, message);
        }
    }

    @Override
    public synchronized void close() {
        connected = false;
        try {
            if (input != null) input.close();
        } catch (Exception ignored) {
        }
        try {
            if (output != null) output.close();
        } catch (Exception ignored) {
        }
        try {
            if (socket != null) socket.close();
        } catch (Exception ignored) {
        }
    }
}
