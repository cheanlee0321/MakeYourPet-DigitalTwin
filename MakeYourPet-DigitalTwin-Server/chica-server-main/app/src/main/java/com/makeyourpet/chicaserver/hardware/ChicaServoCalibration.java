package com.makeyourpet.chicaserver.hardware;

import java.util.Locale;
import java.util.Scanner;

/**
 * Parses the servo model from chica.config — per-servo calibration
 * ({@code [name] [pin] [-45 usec] [+45 usec]}), the mechanical attach angles
 * ({@code COXA/FEMUR/TIBIA_ATTACH_ANGLE}), and the servo pin map — into the flat
 * arrays consumed by {@link com.makeyourpet.chicaserver.gait.ChicaGaitEngine#setServoConfig}.
 *
 * Mirrors the original config parser {@code a2.n} + the angle->pulse fields it
 * populates in {@code z0.h}. Leg indexing matches the engine: L1..L3 = 0..2,
 * R1..R3 = 3..5; joint = servo-name suffix 1..3 -> 0..2.
 */
public final class ChicaServoCalibration {
    private static final String[] LEG_NAMES = {"L1", "L2", "L3", "R1", "R2", "R3"};
    private static final int[][] DEFAULT_PINS = {
            {15, 16, 17}, {9, 10, 11}, {3, 4, 5},
            {12, 13, 14}, {6, 7, 8}, {0, 1, 2}
    };

    // [leg][joint][0]=-45 usec, [1]=+45 usec
    public final int[][][] calibration = new int[6][3][2];
    public final int[][] pin = new int[6][3];
    public final double[] coxaAttach = {-8.0, 0.0, 8.0, -8.0, 0.0, 8.0};
    public double femurAttach = 35.0;
    public double tibiaAttach = 68.0;

    private ChicaServoCalibration() {
        for (int leg = 0; leg < 6; leg++) {
            for (int joint = 0; joint < 3; joint++) {
                calibration[leg][joint][0] = 2000;
                calibration[leg][joint][1] = 1000;
                pin[leg][joint] = DEFAULT_PINS[leg][joint];
            }
        }
    }

    public static ChicaServoCalibration parse(String config) {
        ChicaServoCalibration out = new ChicaServoCalibration();
        if (config == null) return out;
        try (Scanner scanner = new Scanner(config)) {
            scanner.useLocale(Locale.ENGLISH);
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine().trim();
                if (line.isEmpty() || line.startsWith("#")) continue;
                try {
                    if (line.startsWith("COXA_ATTACH_ANGLE")) {
                        double d = firstValue(line);
                        double n = -d;
                        out.coxaAttach[0] = d; out.coxaAttach[1] = 0.0; out.coxaAttach[2] = n;
                        out.coxaAttach[3] = d; out.coxaAttach[4] = 0.0; out.coxaAttach[5] = n;
                    } else if (line.startsWith("FEMUR_ATTACH_ANGLE")) {
                        out.femurAttach = firstValue(line);
                    } else if (line.startsWith("TIBIA_ATTACH_ANGLE")) {
                        out.tibiaAttach = firstValue(line);
                    } else if (line.startsWith("L") || line.startsWith("R")) {
                        out.parseServoLine(line);
                    }
                } catch (RuntimeException ignored) {
                    // skip malformed line, keep defaults for that entry
                }
            }
        }
        return out;
    }

    private void parseServoLine(String line) {
        for (int leg = 0; leg < 6; leg++) {
            for (int joint = 0; joint < 3; joint++) {
                if (line.startsWith(LEG_NAMES[leg] + (joint + 1))) {
                    try (Scanner s = new Scanner(line)) {
                        s.useLocale(Locale.ENGLISH);
                        s.next();                       // servo name
                        pin[leg][joint] = parsePin(s.next());
                        calibration[leg][joint][0] = s.nextInt();   // -45 usec
                        calibration[leg][joint][1] = s.nextInt();   // +45 usec
                    }
                    return;
                }
            }
        }
    }

    private static int parsePin(String token) {
        // Pins are like P15 / S11; the engine pin map uses the numeric channel.
        if (token == null) return 0;
        token = token.trim();
        if (token.isEmpty()) return 0;
        try {
            char prefix = Character.toUpperCase(token.charAt(0));
            if ((prefix == 'P' || prefix == 'S') && token.length() > 1) {
                return Integer.parseInt(token.substring(1).trim());
            }
            return Integer.parseInt(token);
        } catch (NumberFormatException e) {
            return 0;
        }
    }

    private static double firstValue(String line) {
        try (Scanner s = new Scanner(line)) {
            s.useLocale(Locale.ENGLISH);
            s.next();
            return s.nextDouble();
        }
    }

    /** Flatten calibration to [leg*3+joint]*2 + {lo,hi} for the JNI setter. */
    public int[] flatCalibration() {
        int[] out = new int[36];
        for (int leg = 0; leg < 6; leg++) {
            for (int joint = 0; joint < 3; joint++) {
                int idx = leg * 3 + joint;
                out[idx * 2] = calibration[leg][joint][0];
                out[idx * 2 + 1] = calibration[leg][joint][1];
            }
        }
        return out;
    }

    /** Flatten pin map to [leg*3+joint] for the JNI setter. */
    public int[] flatPins() {
        int[] out = new int[18];
        for (int leg = 0; leg < 6; leg++) {
            for (int joint = 0; joint < 3; joint++) {
                out[leg * 3 + joint] = pin[leg][joint];
            }
        }
        return out;
    }
}
