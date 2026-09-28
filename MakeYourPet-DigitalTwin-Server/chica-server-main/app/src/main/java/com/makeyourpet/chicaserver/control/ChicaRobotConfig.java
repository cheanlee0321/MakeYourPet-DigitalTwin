package com.makeyourpet.chicaserver.control;

import java.util.Locale;
import java.util.Scanner;

/**
 * Parses the body geometry and the six MODE_* parameter sets from chica.config,
 * mirroring the original config parser {@code a2.n.f}/{@code a2.n.j} and the
 * {@code z0.h}/{@code z0.g} fields they populate. Defaults match the stock
 * config-2040 so an absent/partial config reproduces the previous hardcoded
 * behavior.
 */
final class ChicaRobotConfig {
    // MODE_* order, matching z0.g.f7084h and ChicaController's mode indexing.
    private static final String[] MODE_NAMES = {
            "MODE_STANDARD", "MODE_RACE", "MODE_OFFROAD",
            "MODE_CUSTOM", "MODE_QUADRUPED", "MODE_BLOCK"
    };

    // Geometry (defaults = stock config-2040 / native makeDefaultConfig).
    double coxaLen = 43.0;
    double femurLen = 80.0;
    double tibiaLen = 134.0;
    double l1ToR1 = 126.0;
    double l1ToL3 = 167.0;
    double l2ToR2 = 163.0;
    double legConnectionZ = -10.0;
    double legSittingZ = -40.0;

    // Per mode: {radius, cornerAngle, elongation, bodyLift, stepLift, speed, animationFactor}
    final double[][] modes = {
            {220.0, 55.0, 1.15, 40.0, 40.0, 1.0, 1.0},
            {210.0, 55.0, 1.20, 35.0, 30.0, 2.0, 0.0},
            {230.0, 55.0, 1.15, 60.0, 99.0, 0.6, 1.0},
            {220.0, 55.0, 1.15, 40.0, 40.0, 1.0, 1.0},
            {220.0, 60.0, 1.00, 45.0, 35.0, 0.8, 1.0},
            {185.0, 30.0, 1.07, -40.0, 80.0, 1.0, 1.0}
    };

    private ChicaRobotConfig() {
    }

    static ChicaRobotConfig parse(String config) {
        ChicaRobotConfig out = new ChicaRobotConfig();
        if (config == null) return out;
        try (Scanner scanner = new Scanner(config)) {
            scanner.useLocale(Locale.ENGLISH);
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine().trim();
                if (line.isEmpty() || line.startsWith("#")) continue;
                try {
                    if (line.startsWith("COXA_LEN")) {
                        out.coxaLen = firstValue(line);
                    } else if (line.startsWith("FEMUR_LEN")) {
                        out.femurLen = firstValue(line);
                    } else if (line.startsWith("TIBIA_LEN")) {
                        out.tibiaLen = firstValue(line);
                    } else if (line.startsWith("L1_TO_R1")) {
                        out.l1ToR1 = firstValue(line);
                    } else if (line.startsWith("L1_TO_L3")) {
                        out.l1ToL3 = firstValue(line);
                    } else if (line.startsWith("L2_TO_R2")) {
                        out.l2ToR2 = firstValue(line);
                    } else if (line.startsWith("LEG_CONNECTION_Z")) {
                        out.legConnectionZ = firstValue(line);
                    } else if (line.startsWith("LEG_SITTING_Z")) {
                        out.legSittingZ = firstValue(line);
                    } else if (line.startsWith("MODE_")) {
                        out.parseModeLine(line);
                    }
                } catch (RuntimeException ignored) {
                    // keep defaults for a malformed line
                }
            }
        }
        return out;
    }

    private void parseModeLine(String line) {
        int index = -1;
        for (int i = 0; i < MODE_NAMES.length; i++) {
            if (line.startsWith(MODE_NAMES[i])) {
                index = i;
                break;
            }
        }
        if (index < 0) return;
        try (Scanner s = new Scanner(line)) {
            s.useLocale(Locale.ENGLISH);
            s.next();                          // mode name
            double radius = s.nextDouble();
            double corner = s.nextDouble();
            double elongation = s.nextDouble();
            double bodyLift = s.nextDouble();
            double stepLift = s.nextDouble();
            s.nextDouble();                    // 6th value unused (matches a2.n.j)
            double speed = s.nextDouble();
            if (speed <= 0.0) speed = 1.0;
            double animationFactor = s.nextDouble();
            modes[index] = new double[] {radius, corner, elongation, bodyLift, stepLift, speed, animationFactor};
        }
    }

    private static double firstValue(String line) {
        try (Scanner s = new Scanner(line)) {
            s.useLocale(Locale.ENGLISH);
            s.next();
            return s.nextDouble();
        }
    }
}
