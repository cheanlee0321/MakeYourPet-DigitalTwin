package com.makeyourpet.chicaserver;

import android.content.Context;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.util.Locale;
import java.util.Scanner;

final class ConfigStore {
    private static final String DEFAULT_ASSET = "config-2040.txt";
    private static final String USER_CONFIG = "chica.config";
    private static final String[] SERVO_PREFIXES = {
            "L11", "L12", "L13",
            "L21", "L22", "L23",
            "L31", "L32", "L33",
            "R11", "R12", "R13",
            "R21", "R22", "R23",
            "R31", "R32", "R33"
    };
    private static final String[] TOUCH_PREFIXES = {
            "TS_L1", "TS_L2", "TS_L3", "TS_R1", "TS_R2", "TS_R3"
    };
    private static final String[] MODE_PREFIXES = {
            "MODE_STANDARD", "MODE_RACE", "MODE_OFFROAD",
            "MODE_CUSTOM", "MODE_QUADRUPED", "MODE_BLOCK"
    };

    private final Context context;
    private String text;

    ConfigStore(Context context) {
        this.context = context.getApplicationContext();
    }

    synchronized String load() {
        if (text != null) return text;
        text = loadDefaultText();
        File userConfig = userConfigFile();
        if (userConfig.exists()) {
            try {
                String candidate = read(new FileInputStream(userConfig));
                // The original only persists chica.config on Save (r8.p(text, true));
                // load never writes. An invalid file falls back to the default in
                // memory without overwriting it on disk.
                if (isOriginalValidConfig(candidate)) {
                    text = candidate;
                }
            } catch (Exception error) {
                System.out.println(error);
            }
        }
        return text;
    }

    synchronized void save(String nextText) {
        String previous = load();
        if (isOriginalValidConfig(nextText)) {
            text = nextText;
        } else {
            text = previous;
            isOriginalValidConfig(text);
        }
        try {
            writeUserConfig(text);
        } catch (Exception error) {
            System.out.println(error);
        }
    }

    private File userConfigFile() {
        return new File(context.getFilesDir(), USER_CONFIG);
    }

    private String loadDefaultText() {
        try {
            return read(context.getAssets().open(DEFAULT_ASSET));
        } catch (Exception error) {
            return "# Unable to load " + DEFAULT_ASSET + "\n" + error;
        }
    }

    private void writeUserConfig(String value) throws Exception {
        if (value == null) return;
        try (FileOutputStream out = new FileOutputStream(userConfigFile())) {
            out.write(value.getBytes("UTF-8"));
            out.flush();
        }
    }

    private static String read(InputStream input) throws Exception {
        try (InputStream in = input; ByteArrayOutputStream out = new ByteArrayOutputStream()) {
            byte[] buffer = new byte[4096];
            int read;
            while ((read = in.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
            return out.toString("UTF-8");
        }
    }

    private static boolean isOriginalValidConfig(String config) {
        if (config == null || config.isEmpty()) return false;
        Scanner scanner = new Scanner(config);
        scanner.useLocale(Locale.ENGLISH);
        int deviceLines = 0;
        try {
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine().trim();
                if (line.isEmpty() || line.startsWith("#")) continue;
                if (line.startsWith("COXA_LEN")
                        || line.startsWith("TIBIA_LEN")
                        || line.startsWith("FEMUR_LEN")
                        || line.startsWith("L1_TO_R1")
                        || line.startsWith("L1_TO_L3")
                        || line.startsWith("L2_TO_R2")
                        || line.startsWith("LEG_CONNECTION_Z")
                        || line.startsWith("LEG_SITTING_Z")
                        || line.startsWith("COXA_ATTACH_ANGLE")
                        || line.startsWith("FEMUR_ATTACH_ANGLE")
                        || line.startsWith("TIBIA_ATTACH_ANGLE")) {
                    readSingleDouble(line);
                } else if (line.startsWith("VOL") || line.startsWith("CUR")) {
                    readAnalogLine(line);
                } else if (line.startsWith("RELAY")) {
                    readRelayLine(line);
                } else if (line.startsWith("WARN_VOL") || line.startsWith("WARN_CUR")) {
                    readWarningLine(line);
                } else if (isKnownModeLine(line)) {
                    readModeLine(line);
                } else if (isServoLine(line)) {
                    readServoLine(line);
                    deviceLines++;
                } else if (isTouchLine(line)) {
                    readTouchLine(line);
                    deviceLines++;
                }
            }
            if (deviceLines >= 18) return true;
            throw new RuntimeException("Duplicate or missing config");
        } catch (Exception error) {
            System.out.println(error);
            return false;
        } finally {
            scanner.close();
        }
    }

    private static boolean isServoLine(String line) {
        if (!line.startsWith("L") && !line.startsWith("R")) return false;
        for (String prefix : SERVO_PREFIXES) {
            if (line.startsWith(prefix)) return true;
        }
        return false;
    }

    private static boolean isTouchLine(String line) {
        if (!line.startsWith("TS_")) return false;
        for (String prefix : TOUCH_PREFIXES) {
            if (line.startsWith(prefix)) return true;
        }
        return false;
    }

    private static boolean isKnownModeLine(String line) {
        if (!line.startsWith("MODE_")) return false;
        for (String prefix : MODE_PREFIXES) {
            if (line.startsWith(prefix)) return true;
        }
        return false;
    }

    private static void readSingleDouble(String line) {
        Scanner s = scanner(line);
        try {
            s.next();
            s.nextDouble();
        } finally {
            s.close();
        }
    }

    private static void readAnalogLine(String line) {
        Scanner s = scanner(line);
        try {
            s.next();
            readPin(s.next());
            s.nextDouble();
            s.nextDouble();
        } finally {
            s.close();
        }
    }

    private static void readRelayLine(String line) {
        Scanner s = scanner(line);
        try {
            s.next();
            readPin(s.next());
        } finally {
            s.close();
        }
    }

    private static void readWarningLine(String line) {
        Scanner s = scanner(line);
        try {
            s.next();
            s.nextDouble();
            s.nextDouble();
            s.nextDouble();
            s.nextInt();
        } finally {
            s.close();
        }
    }

    private static void readModeLine(String line) {
        Scanner s = scanner(line);
        try {
            s.next();
            for (int i = 0; i < 8; i++) {
                s.nextDouble();
            }
        } finally {
            s.close();
        }
    }

    private static void readServoLine(String line) {
        Scanner s = scanner(line);
        try {
            s.next();
            readPin(s.next());
            s.nextInt();
            s.nextInt();
        } finally {
            s.close();
        }
    }

    private static void readTouchLine(String line) {
        Scanner s = scanner(line);
        try {
            s.next();
            readPin(s.next());
        } finally {
            s.close();
        }
    }

    private static void readPin(String token) {
        if (token == null) throw new IllegalArgumentException("Bad pin");
        token = token.trim();
        if (token.length() < 2) throw new IllegalArgumentException("Bad pin");
        char prefix = Character.toUpperCase(token.charAt(0));
        if (prefix != 'P' && prefix != 'S') throw new IllegalArgumentException("Bad pin");
        Integer.parseInt(token.substring(1).trim());
    }

    private static Scanner scanner(String line) {
        Scanner scanner = new Scanner(line);
        scanner.useLocale(Locale.ENGLISH);
        return scanner;
    }
}
