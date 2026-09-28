package com.makeyourpet.chicaserver.hardware;

public final class ServoPacketEncoder {
    private ServoPacketEncoder() {
    }

    public static byte[] servo2040Pulses(int[] pulses) {
        byte[] frame = new byte[39];
        servo2040Pulses(pulses, frame);
        return frame;
    }

    public static void servo2040Pulses(int[] pulses, byte[] frame) {
        if (frame == null || frame.length != 39) throw new IllegalArgumentException("Servo2040 frame must be 39 bytes");
        frame[0] = (byte) 0xd3;
        frame[1] = 0;
        frame[2] = 18;
        for (int i = 0; i < 18; i++) {
            int pulse = (pulses != null && i < pulses.length) ? pulses[i] : 0;
            int offset = (i * 2) + 3;
            frame[offset] = (byte) (pulse & 0x7f);
            frame[offset + 1] = (byte) ((pulse >> 7) & 0x7f);
        }
    }

    public static byte[] pololuPulses(int[] pulses) {
        byte[] frame = new byte[39];
        pololuPulses(pulses, frame);
        return frame;
    }

    public static void pololuPulses(int[] pulses, byte[] frame) {
        if (frame == null || frame.length != 39) throw new IllegalArgumentException("Pololu frame must be 39 bytes");
        frame[0] = (byte) 0x9f;
        frame[1] = 18;
        frame[2] = 0;
        for (int i = 0; i < 18; i++) {
            int pulse = (pulses != null && i < pulses.length) ? pulses[i] : 0;
            int quarterMicroseconds = pulse * 4;
            int offset = (i * 2) + 3;
            frame[offset] = (byte) (quarterMicroseconds & 0x7f);
            frame[offset + 1] = (byte) ((quarterMicroseconds >> 7) & 0x7f);
        }
    }

    public static byte[] servo2040DigitalOut(int pin, boolean enabled) {
        int value = enabled ? 1 : 0;
        return new byte[] {
                (byte) 0xd3,
                (byte) pin,
                1,
                (byte) (value & 0x7f),
                (byte) ((value >> 7) & 0x7f)
        };
    }

    public static byte[] pololuDigitalOut(int pin, boolean enabled) {
        int target = enabled ? 7000 : 5000;
        return new byte[] {
                (byte) 0x84,
                (byte) pin,
                (byte) (target & 0x7f),
                (byte) ((target >> 7) & 0x7f)
        };
    }
}
