package com.makeyourpet.chicaserver.control;

import java.util.Locale;

public final class SurfaceStatus {
    public static final double[] UNKNOWN_LEG_TOUCHES = {
            Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN
    };

    // Immediate V/I text-color zones (matches the original's z0.d threshold
    // colouring): OK = green, WARN = yellow, CRITICAL = red. Distinct from the
    // duration-gated voltageWarning/currentWarning flags, which drive the warning
    // boxes only.
    public static final int ZONE_OK = 0;
    public static final int ZONE_WARN = 1;
    public static final int ZONE_CRITICAL = 2;

    public final double voltage;
    public final double current;
    public final double bps;
    public final String ipAddress;
    public final double[] legTouches;
    public final double primaryX;
    public final double primaryY;
    public final double secondaryX;
    public final double secondaryY;
    public final double forward;
    public final double strafe;
    public final double turn;
    public final boolean voltageWarning;
    public final boolean currentWarning;
    public final int voltageZone;
    public final int currentZone;
    // Status-string BPS is `bps` (board-poll rate). panelBps is the separate
    // on-screen joystick-panel number (orientation-sensor fusion rate).
    public final double panelBps;

    public SurfaceStatus(double voltage, double current, double bps, String ipAddress,
            double primaryX, double primaryY, double secondaryX, double secondaryY,
            double forward, double strafe, double turn,
            boolean voltageWarning, boolean currentWarning,
            int voltageZone, int currentZone, double panelBps) {
        this(voltage, current, bps, ipAddress, UNKNOWN_LEG_TOUCHES,
                primaryX, primaryY, secondaryX, secondaryY, forward, strafe, turn,
                voltageWarning, currentWarning, voltageZone, currentZone, panelBps);
    }

    public SurfaceStatus(double voltage, double current, double bps, String ipAddress,
            double[] legTouches,
            double primaryX, double primaryY, double secondaryX, double secondaryY,
            double forward, double strafe, double turn,
            boolean voltageWarning, boolean currentWarning,
            int voltageZone, int currentZone, double panelBps) {
        this.voltage = voltage;
        this.current = current;
        this.bps = bps;
        this.ipAddress = ipAddress == null ? "" : ipAddress;
        this.legTouches = legTouches == null ? UNKNOWN_LEG_TOUCHES : legTouches.clone();
        this.primaryX = primaryX;
        this.primaryY = primaryY;
        this.secondaryX = secondaryX;
        this.secondaryY = secondaryY;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.voltageWarning = voltageWarning;
        this.currentWarning = currentWarning;
        this.voltageZone = voltageZone;
        this.currentZone = currentZone;
        this.panelBps = panelBps;
    }

    public String toOriginalPayload() {
        return "BPS=" + String.format(Locale.ENGLISH, "% 3d", (int) bps)
                + "|V=" + number(voltage)
                + "|I=" + number(current)
                + "|IP=" + ipAddress
                + "|LEGS=" + legs();
    }

    private static String number(double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) return "---";
        return String.format(Locale.ENGLISH, "% 3.3f", value);
    }

    private String legs() {
        StringBuilder text = new StringBuilder();
        for (int i = 0; i < 6; i++) {
            double value = i < legTouches.length ? legTouches[i] : Double.NaN;
            text.append(!Double.isNaN(value) && value > 0.5d ? "x" : "-");
        }
        return text.toString();
    }
}
