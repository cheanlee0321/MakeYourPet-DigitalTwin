package com.makeyourpet.chicaserver.hardware;

import android.hardware.usb.UsbDeviceConnection;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import java.io.IOException;
import java.util.Arrays;

final class UsbSerialServoBackend implements ServoBackend {
    enum BoardProtocol {
        SERVO2040,
        POLOLU
    }

    private static final int TIMEOUT_MS = 200;

    private final BoardProtocol protocol;
    private final UsbSerialPort primaryPort;
    private final UsbSerialPort secondaryPort;
    private final UsbDeviceConnection connection;
    private final ChicaConfigPins pins;
    private volatile boolean connected;
    private volatile boolean fault;
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
    private byte[] primaryPollFrame;
    private byte[] secondaryPollFrame;
    private byte[] servo2040PollFrame;

    UsbSerialServoBackend(BoardProtocol protocol, UsbSerialPort primaryPort,
            UsbSerialPort secondaryPort, ChicaConfigPins pins, UsbDeviceConnection connection) {
        this.protocol = protocol;
        this.primaryPort = primaryPort;
        this.secondaryPort = secondaryPort;
        this.connection = connection;
        this.pins = pins;
        this.connected = primaryPort != null;
    }

    @Override
    public String name() {
        return protocol == BoardProtocol.SERVO2040 ? "servo2040-usb" : "pololu-usb";
    }

    @Override
    public boolean isConnected() {
        return connected;
    }

    @Override
    public boolean hasFault() {
        return fault;
    }

    @Override
    public synchronized void restartPort() {
        // Mirrors z0.i.b(): close and reopen the port(s) with the same parameters
        // used at open time, re-running the Servo2040 DTR/drain handshake. Clears
        // the fault latch on success so the watchdog stops retrying.
        try {
            reopen(primaryPort);
            if (secondaryPort != null) reopen(secondaryPort);
            if (protocol == BoardProtocol.SERVO2040 && primaryPort != null) {
                primaryPort.setDTR(true);
                Thread.sleep(200L);
                drain(primaryPort);
            }
            connected = primaryPort != null;
            fault = false;
        } catch (Exception error) {
            // Reopen failed; leave fault set so the watchdog retries.
            connected = false;
        }
    }

    private void reopen(UsbSerialPort port) throws Exception {
        if (port == null || connection == null) return;
        try {
            port.close();
        } catch (Exception ignored) {
        }
        port.open(connection);
        port.setParameters(115200, 8, 1, UsbSerialPort.PARITY_NONE);
    }

    private static void drain(UsbSerialPort port) {
        byte[] buffer = new byte[1024];
        try {
            while (port.read(buffer, 100) > 0) {
                // Discard startup bytes, matching the open-time drain.
            }
        } catch (Exception ignored) {
        }
    }

    @Override
    public synchronized void stageServoPulses(int[] pulses) {
        if (stagedServoFrame == null) stagedServoFrame = new byte[39];
        if (protocol == BoardProtocol.SERVO2040) {
            ServoPacketEncoder.servo2040Pulses(pulses, stagedServoFrame);
        } else {
            ServoPacketEncoder.pololuPulses(pulses, stagedServoFrame);
        }
    }

    @Override
    public synchronized void flushServoPulses() {
        if (stagedServoFrame != null) write(primaryPort, stagedServoFrame);
    }

    @Override
    public synchronized void setRelay(boolean enabled) {
        relayEnabled = enabled;
        ChicaConfigPins.UsbPin relay = pins.relayPin;
        if (relay == null) return;
        UsbSerialPort port = relay.primaryBoard ? primaryPort : secondaryPort;
        if (protocol == BoardProtocol.SERVO2040) {
            write(port, ServoPacketEncoder.servo2040DigitalOut(relay.pin, enabled));
        } else {
            // The original Pololu backend inverted relay state before sending target pulses.
            write(port, ServoPacketEncoder.pololuDigitalOut(relay.pin, !enabled));
        }
    }

    @Override
    public synchronized void refreshTelemetry() {
        if (protocol == BoardProtocol.SERVO2040) {
            refreshServo2040Telemetry();
        } else {
            refreshPololuTelemetry(true);
            refreshPololuTelemetry(false);
        }
        voltage = originalVoltage();
        current = originalCurrent();
        for (int i = 0; i < legTouches.length; i++) {
            int raw = touchRaw[i];
            legTouches[i] = raw == Integer.MIN_VALUE ? Double.NaN : raw / 1024.0d;
        }
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

    private void write(UsbSerialPort port, byte[] frame) {
        if (port == null) {
            connected = false;
            return;
        }
        try {
            port.write(frame, protocol == BoardProtocol.SERVO2040 ? 0 : TIMEOUT_MS);
            connected = true;
        } catch (IOException error) {
            connected = false;
            fault = true;
        }
    }

    private void refreshPololuTelemetry(boolean primaryBoard) {
        int count = countConfiguredPins(primaryBoard);
        if (count == 0) return;
        int length = Math.max(16, count * 2);
        byte[] request = primaryBoard ? primaryPollFrame : secondaryPollFrame;
        if (request == null || request.length != length) {
            request = buildPololuPollFrame(primaryBoard, length);
            if (primaryBoard) {
                primaryPollFrame = request;
            } else {
                secondaryPollFrame = request;
            }
        }
        byte[] reply = new byte[length];
        UsbSerialPort port = primaryBoard ? primaryPort : secondaryPort;
        if (!writeAndReadExact(port, request, reply, TIMEOUT_MS)) {
            clearRawTelemetry();
            return;
        }
        int index = 0;
        for (int leg = 0; leg < pins.touchPins.length; leg++) {
            ChicaConfigPins.UsbPin pin = pins.touchPins[leg];
            if (pin != null && pin.primaryBoard == primaryBoard) {
                touchRaw[leg] = rawPololu(reply, index);
                index++;
            }
        }
        if (isOnBoard(pins.volPin, primaryBoard)) {
            volRaw = rawPololu(reply, index);
            index++;
        }
        if (isOnBoard(pins.curPin, primaryBoard)) {
            curRaw = rawPololu(reply, index);
        }
    }

    private void refreshServo2040Telemetry() {
        int count = countServo2040Pins();
        if (count == 0) return;
        int requestLength = count * 3;
        if (servo2040PollFrame == null || servo2040PollFrame.length != requestLength) {
            servo2040PollFrame = buildServo2040PollFrame(requestLength);
        }
        byte[] reply = new byte[count * 5];
        if (!writeAndReadExact(primaryPort, servo2040PollFrame, reply, 0)) {
            clearRawTelemetry();
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
        if (isOnBoard(pins.volPin, true)) {
            volRaw = rawServo2040(reply, index);
            index++;
        }
        if (isOnBoard(pins.curPin, true)) {
            curRaw = rawServo2040(reply, index);
        }
    }

    private boolean writeAndReadExact(UsbSerialPort port, byte[] request, byte[] reply, int timeoutMs) {
        if (port == null) {
            connected = false;
            return false;
        }
        try {
            port.write(request, timeoutMs);
            int received = readExact(port, reply, timeoutMs);
            connected = received == reply.length;
            return connected;
        } catch (Exception error) {
            connected = false;
            fault = true;
            return false;
        }
    }

    private int readExact(UsbSerialPort port, byte[] reply, int timeoutMs) throws IOException {
        if (protocol == BoardProtocol.POLOLU) {
            int read = port.read(reply, timeoutMs);
            return read < 0 ? 0 : read;
        }
        byte[] buffer = new byte[256];
        int received = 0;
        while (true) {
            int read = port.read(buffer, 100);
            if (read == 0) break;
            for (int i = 0; i < read; i++) {
                if (received < reply.length) reply[received] = buffer[i];
                received++;
            }
            if (received >= reply.length) break;
        }
        return received;
    }

    private byte[] buildPololuPollFrame(boolean primaryBoard, int length) {
        byte[] frame = new byte[length];
        int offset = 0;
        for (ChicaConfigPins.UsbPin pin : pins.touchPins) {
            if (pin != null && pin.primaryBoard == primaryBoard) {
                offset = appendPololuRead(frame, offset, pin.pin);
            }
        }
        if (isOnBoard(pins.volPin, primaryBoard)) {
            offset = appendPololuRead(frame, offset, pins.volPin.pin.pin);
        }
        if (isOnBoard(pins.curPin, primaryBoard)) {
            offset = appendPololuRead(frame, offset, pins.curPin.pin.pin);
        }
        while (offset < frame.length) {
            frame[offset++] = frame[0];
            frame[offset++] = frame[1];
        }
        return frame;
    }

    private byte[] buildServo2040PollFrame(int length) {
        byte[] frame = new byte[length];
        int offset = 0;
        for (ChicaConfigPins.UsbPin pin : pins.touchPins) {
            if (pin != null && pin.primaryBoard) {
                offset = appendServo2040Read(frame, offset, pin.pin);
            }
        }
        if (isOnBoard(pins.volPin, true)) {
            offset = appendServo2040Read(frame, offset, pins.volPin.pin.pin);
        }
        if (isOnBoard(pins.curPin, true)) {
            offset = appendServo2040Read(frame, offset, pins.curPin.pin.pin);
        }
        return frame;
    }

    private int appendPololuRead(byte[] frame, int offset, int pin) {
        frame[offset++] = (byte) 0x90;
        frame[offset++] = (byte) pin;
        return offset;
    }

    private int appendServo2040Read(byte[] frame, int offset, int pin) {
        frame[offset++] = (byte) 0xc7;
        frame[offset++] = (byte) pin;
        frame[offset++] = 1;
        return offset;
    }

    private int countConfiguredPins(boolean primaryBoard) {
        int count = 0;
        for (ChicaConfigPins.UsbPin pin : pins.touchPins) {
            if (pin != null && pin.primaryBoard == primaryBoard) count++;
        }
        if (isOnBoard(pins.volPin, primaryBoard)) count++;
        if (isOnBoard(pins.curPin, primaryBoard)) count++;
        return count;
    }

    private int countServo2040Pins() {
        int count = 0;
        for (ChicaConfigPins.UsbPin pin : pins.touchPins) {
            if (pin != null && pin.primaryBoard) count++;
        }
        if (isOnBoard(pins.volPin, true)) count++;
        if (isOnBoard(pins.curPin, true)) count++;
        return count;
    }

    private static boolean isOnBoard(ChicaConfigPins.AnalogPin pin, boolean primaryBoard) {
        return pin != null && pin.pin != null && pin.pin.primaryBoard == primaryBoard;
    }

    private static int rawPololu(byte[] reply, int index) {
        int offset = index * 2;
        if (offset + 1 >= reply.length) return Integer.MIN_VALUE;
        return ((reply[offset + 1] & 0xff) * 255) + (reply[offset] & 0xff);
    }

    private static int rawServo2040(byte[] reply, int index) {
        int offset = index * 5;
        if (offset + 4 >= reply.length) return Integer.MIN_VALUE;
        return (reply[offset + 3] & 0x7f) | ((reply[offset + 4] & 0x7f) << 7);
    }

    private double originalVoltage() {
        if (volRaw == Integer.MIN_VALUE) return Double.NaN;
        if (protocol == BoardProtocol.SERVO2040) {
            return volRaw / 310.29998779296875d;
        }
        if (pins.volPin == null || pins.volPin.oneVoltage == pins.volPin.zeroVoltage) return Double.NaN;
        return (((volRaw * 5.0d) / 1023.0d) - pins.volPin.zeroVoltage)
                / (pins.volPin.oneVoltage - pins.volPin.zeroVoltage);
    }

    private double originalCurrent() {
        if (curRaw == Integer.MIN_VALUE) return Double.NaN;
        if (protocol == BoardProtocol.SERVO2040) {
            return (curRaw - 512.0d) * 0.08139999955892563d;
        }
        if (pins.curPin == null || pins.curPin.oneVoltage == pins.curPin.zeroVoltage) return Double.NaN;
        return (((curRaw * 5.0d) / 1023.0d) - pins.curPin.zeroVoltage)
                / (pins.curPin.oneVoltage - pins.curPin.zeroVoltage);
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

    @Override
    public synchronized void close() {
        connected = false;
        try {
            if (primaryPort != null) primaryPort.close();
        } catch (Exception ignored) {
        }
        try {
            if (secondaryPort != null) secondaryPort.close();
        } catch (Exception ignored) {
        }
        try {
            if (connection != null) connection.close();
        } catch (Exception ignored) {
        }
    }
}
