package com.makeyourpet.chicaserver.hardware;

import java.util.Arrays;

public final class VirtualServoBackend implements ServoBackend {
    private static final int[] FEMUR_PINS = {16, 10, 4, 13, 7, 1};
    private static final int[] TOUCH_THRESHOLDS = {8, 10, 12, 8, 10, 12};
    // The virtual-touch fixture build also simulates the battery ADC, reporting
    // a steady 7.4 V / 7.4 A on both telemetry channels (the controller's EWMA
    // blend converges these to the original's " 7.400" status readout). Without
    // the fixture there is no simulated hardware, so telemetry reads NaN ("---").
    private static final double VIRTUAL_BATTERY = 7.4d;

    private final int[] lastPulses = new int[18];
    private final int[] previousFemurPulses = new int[6];
    private final int[] raiseBaselineFemurPulses = new int[6];
    private final double[] legTouches = {
            Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN
    };
    private boolean relayEnabled;
    private boolean virtualTouchFixture;
    private boolean hasPreviousPulses;
    private boolean hasRaiseBaseline;
    private double virtualVoltage = VIRTUAL_BATTERY;
    private double virtualCurrent = VIRTUAL_BATTERY;

    public VirtualServoBackend() {
        Arrays.fill(lastPulses, 1500);
    }

    @Override
    public String name() {
        return "virtual";
    }

    @Override
    public boolean isConnected() {
        // Only "connected" when simulating a board (the virtual-touch fixture);
        // with no hardware there is nothing to poll, so the status BPS stays 0.
        return virtualTouchFixture;
    }

    @Override
    public synchronized void stageServoPulses(int[] pulses) {
        if (pulses == null) return;
        System.arraycopy(pulses, 0, lastPulses, 0, Math.min(pulses.length, lastPulses.length));
        noteVirtualTouchPulses();
    }

    @Override
    public void flushServoPulses() {
        // Staging is the complete virtual-device operation.
    }

    @Override
    public synchronized void setRelay(boolean enabled) {
        relayEnabled = enabled;
    }

    @Override
    public synchronized void refreshTelemetry() {
        if (!virtualTouchFixture) return;
        if (!hasRaiseBaseline) {
            Arrays.fill(legTouches, 0.0d);
            return;
        }
        for (int leg = 0; leg < legTouches.length; leg++) {
            int pulse = previousFemurPulses[leg];
            int baseline = raiseBaselineFemurPulses[leg];
            int delta = leg < 3 ? pulse - baseline : baseline - pulse;
            legTouches[leg] = delta >= TOUCH_THRESHOLDS[leg] ? 1.0d : 0.0d;
        }
    }

    @Override
    public synchronized void setVirtualTouchFixture(boolean enabled) {
        virtualTouchFixture = enabled;
        hasRaiseBaseline = false;
        virtualVoltage = VIRTUAL_BATTERY;
        virtualCurrent = VIRTUAL_BATTERY;
        for (int leg = 0; leg < previousFemurPulses.length; leg++) {
            previousFemurPulses[leg] = lastPulses[FEMUR_PINS[leg]];
        }
        hasPreviousPulses = enabled;
        Arrays.fill(legTouches, enabled ? 0.0d : Double.NaN);
    }

    @Override
    public synchronized void setVirtualTelemetry(double voltage, double current) {
        virtualTouchFixture = true;
        virtualVoltage = voltage;
        virtualCurrent = current;
        if (!hasPreviousPulses) {
            for (int leg = 0; leg < previousFemurPulses.length; leg++) {
                previousFemurPulses[leg] = lastPulses[FEMUR_PINS[leg]];
            }
            hasPreviousPulses = true;
        }
        if (!hasRaiseBaseline) {
            Arrays.fill(legTouches, 0.0d);
        }
    }

    @Override
    public synchronized double voltage() {
        return virtualTouchFixture ? virtualVoltage : Double.NaN;
    }

    @Override
    public synchronized double current() {
        return virtualTouchFixture ? virtualCurrent : Double.NaN;
    }

    @Override
    public synchronized double[] legTouches() {
        return legTouches.clone();
    }

    public synchronized int[] lastPulses() {
        return lastPulses.clone();
    }

    public synchronized boolean relayEnabled() {
        return relayEnabled;
    }

    private void noteVirtualTouchPulses() {
        if (!virtualTouchFixture) return;
        if (isCalibrationRaiseFrame()) {
            for (int leg = 0; leg < raiseBaselineFemurPulses.length; leg++) {
                raiseBaselineFemurPulses[leg] = lastPulses[FEMUR_PINS[leg]];
            }
            hasRaiseBaseline = true;
        }
        for (int leg = 0; leg < previousFemurPulses.length; leg++) {
            previousFemurPulses[leg] = lastPulses[FEMUR_PINS[leg]];
        }
        hasPreviousPulses = true;
    }

    private boolean isCalibrationRaiseFrame() {
        if (!hasPreviousPulses) return false;
        for (int leg = 0; leg < FEMUR_PINS.length; leg++) {
            int previous = previousFemurPulses[leg];
            int current = lastPulses[FEMUR_PINS[leg]];
            int raiseDelta = leg < 3 ? previous - current : current - previous;
            if (raiseDelta < 15) return false;
        }
        return true;
    }
}
