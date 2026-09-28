package com.makeyourpet.chicaserver.hardware;

public interface ServoBackend extends AutoCloseable {
    String name();

    boolean isConnected();

    /**
     * True once a serial operation has failed and the port needs reopening.
     * Mirrors the original z0.i fault flag {@code c}, latched by z0.i.k() on a
     * negative op result. Non-serial backends never fault.
     */
    default boolean hasFault() {
        return false;
    }

    /**
     * Reopen the serial port after a fault, clearing {@link #hasFault()} on
     * success. Mirrors the original z0.i.b() port (re)initialization invoked by
     * the hardware-monitor worker. No-op for non-serial backends.
     */
    default void restartPort() {
    }

    /** Stage the latest 18-channel target without performing device I/O. */
    void stageServoPulses(int[] pulses);

    /** Transmit the most recently staged target. */
    void flushServoPulses();

    /** Immediate helper retained for protocol tests and non-scheduled callers. */
    default void setServoPulses(int[] pulses) {
        stageServoPulses(pulses);
        flushServoPulses();
    }

    void setRelay(boolean enabled);

    default void beep() {
    }

    default void refreshTelemetry() {
    }

    default void setVirtualTouchFixture(boolean enabled) {
    }

    default void setVirtualTelemetry(double voltage, double current) {
    }

    double voltage();

    double current();

    default double[] legTouches() {
        return new double[] {
                Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN
        };
    }

    @Override
    default void close() {
    }
}
