package com.makeyourpet.chicaserver.hardware;

import java.util.Locale;
import java.util.Scanner;

public final class ChicaConfigPins {
    private static final String[] TOUCH_SENSOR_NAMES = {
            "TS_L1", "TS_L2", "TS_L3", "TS_R1", "TS_R2", "TS_R3"
    };

    public final UsbPin relayPin;
    public final UsbPin[] touchPins;
    public final AnalogPin curPin;
    public final AnalogPin volPin;

    private ChicaConfigPins(UsbPin relayPin, UsbPin[] touchPins,
            AnalogPin curPin, AnalogPin volPin) {
        this.relayPin = relayPin;
        this.touchPins = touchPins != null ? touchPins.clone() : new UsbPin[6];
        this.curPin = curPin;
        this.volPin = volPin;
    }

    public static ChicaConfigPins parse(String config) {
        UsbPin relay = null;
        UsbPin[] touch = new UsbPin[6];
        AnalogPin cur = null;
        AnalogPin vol = null;
        try (Scanner scanner = new Scanner(config == null ? "" : config)) {
            scanner.useLocale(Locale.ENGLISH);
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine().trim();
                if (line.isEmpty() || line.startsWith("#")) continue;
                try (Scanner lineScanner = new Scanner(line)) {
                    lineScanner.useLocale(Locale.ENGLISH);
                    if (!lineScanner.hasNext()) {
                        continue;
                    }
                    String key = lineScanner.next();
                    if (line.startsWith("RELAY")) {
                        if (lineScanner.hasNext()) relay = UsbPin.parse(lineScanner.next());
                    } else if (line.startsWith("TS_")) {
                        int leg = touchLegIndex(key);
                        if (leg >= 0 && lineScanner.hasNext()) touch[leg] = UsbPin.parse(lineScanner.next());
                    } else if (line.startsWith("CUR")) {
                        if (lineScanner.hasNext()) {
                            UsbPin pin = UsbPin.parse(lineScanner.next());
                            double zero = lineScanner.hasNextDouble() ? lineScanner.nextDouble() : 0.0d;
                            double one = lineScanner.hasNextDouble() ? lineScanner.nextDouble() : 0.0d;
                            cur = new AnalogPin(pin, zero, one);
                        }
                    } else if (line.startsWith("VOL")) {
                        if (lineScanner.hasNext()) {
                            UsbPin pin = UsbPin.parse(lineScanner.next());
                            double zero = lineScanner.hasNextDouble() ? lineScanner.nextDouble() : 0.0d;
                            double one = lineScanner.hasNextDouble() ? lineScanner.nextDouble() : 0.0d;
                            vol = new AnalogPin(pin, zero, one);
                        }
                    }
                }
            }
        }
        return new ChicaConfigPins(relay, touch, cur, vol);
    }

    private static int touchLegIndex(String key) {
        for (int i = 0; i < TOUCH_SENSOR_NAMES.length; i++) {
            if (TOUCH_SENSOR_NAMES[i].equals(key)) return i;
        }
        return -1;
    }

    public boolean requiresSecondaryBoard() {
        if (relayPin != null && !relayPin.primaryBoard) return true;
        for (UsbPin pin : touchPins) {
            if (pin != null && !pin.primaryBoard) return true;
        }
        if (curPin != null && curPin.pin != null && !curPin.pin.primaryBoard) return true;
        return volPin != null && volPin.pin != null && !volPin.pin.primaryBoard;
    }

    public static final class UsbPin {
        public final int pin;
        public final boolean primaryBoard;

        private UsbPin(int pin, boolean primaryBoard) {
            this.pin = pin;
            this.primaryBoard = primaryBoard;
        }

        public static UsbPin parse(String text) {
            if (text == null) return null;
            text = text.trim();
            if (text.length() < 2) return null;
            char prefix = Character.toUpperCase(text.charAt(0));
            if (prefix != 'P' && prefix != 'S') return null;
            try {
                return new UsbPin(Integer.parseInt(text.substring(1).trim()), prefix == 'P');
            } catch (NumberFormatException e) {
                return null;
            }
        }
    }

    public static final class AnalogPin {
        public final UsbPin pin;
        public final double zeroVoltage;
        public final double oneVoltage;

        private AnalogPin(UsbPin pin, double zeroVoltage, double oneVoltage) {
            this.pin = pin;
            this.zeroVoltage = zeroVoltage;
            this.oneVoltage = oneVoltage;
        }
    }
}
