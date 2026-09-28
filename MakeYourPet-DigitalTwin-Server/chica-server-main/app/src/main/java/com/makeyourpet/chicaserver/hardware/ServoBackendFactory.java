package com.makeyourpet.chicaserver.hardware;

import android.content.Context;
import android.os.Build;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;
import com.hoho.android.usbserial.driver.CdcAcmSerialDriver;
import com.hoho.android.usbserial.driver.ProbeTable;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import java.util.List;

public final class ServoBackendFactory {
    private static final String TAG = "CHICA_USB";

    private ServoBackendFactory() {
    }

    public static ServoBackend open(Context context, String configText) {
        ChicaConfigPins pins = ChicaConfigPins.parse(configText);
        UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        if (usbManager == null) return new VirtualServoBackend();
        logDevices(usbManager);

        ServoBackend pololu = openPololu(usbManager, pins);
        if (pololu != null) return pololu;

        ServoBackend servo2040 = openServo2040(usbManager, pins);
        if (servo2040 != null) return servo2040;

        if (isProbablyEmulator()) {
            ServoBackend tcpFake = SocketServo2040Backend.openIfAvailable("10.0.2.2", 18712, pins);
            if (tcpFake != null) {
                Log.i(TAG, "using " + tcpFake.name());
                return tcpFake;
            }
        }

        ServoBackend ttyServo2040 = TtyServo2040Backend.openIfAvailable("/dev/ttyUSB0", pins);
        if (ttyServo2040 != null) {
            Log.i(TAG, "using " + ttyServo2040.name());
            return ttyServo2040;
        }

        return new VirtualServoBackend();
    }

    private static ServoBackend openPololu(UsbManager usbManager, ChicaConfigPins pins) {
        ProbeTable table = new ProbeTable();
        table.addProduct(8187, 137, CdcAcmSerialDriver.class);
        table.addProduct(8187, 138, CdcAcmSerialDriver.class);
        table.addProduct(8187, 139, CdcAcmSerialDriver.class);
        table.addProduct(8187, 140, CdcAcmSerialDriver.class);
        List<UsbSerialDriver> drivers = new UsbSerialProber(table).findAllDrivers(usbManager);
        logDrivers("pololu", drivers);
        if (drivers.isEmpty()) return null;
        UsbSerialDriver driver = drivers.get(0);
        UsbDeviceConnection connection = usbManager.openDevice(driver.getDevice());
        if (connection == null) return null;
        try {
            List<UsbSerialPort> ports = driver.getPorts();
            if (ports.isEmpty()) {
                connection.close();
                return null;
            }
            UsbSerialPort primary = ports.get(0);
            UsbSerialPort secondary = ports.size() > 1 ? ports.get(1) : null;
            if (pins.requiresSecondaryBoard() && secondary == null) {
                connection.close();
                return null;
            }
            openPort(primary, connection);
            if (secondary != null) openPort(secondary, connection);
            return new UsbSerialServoBackend(UsbSerialServoBackend.BoardProtocol.POLOLU,
                    primary, secondary, pins, connection);
        } catch (Exception error) {
            try {
                connection.close();
            } catch (Exception ignored) {
            }
            return null;
        }
    }

    private static ServoBackend openServo2040(UsbManager usbManager, ChicaConfigPins pins) {
        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager);
        logDrivers("servo2040", drivers);
        if (drivers.isEmpty()) return null;
        UsbSerialDriver driver = drivers.get(0);
        UsbDeviceConnection connection = usbManager.openDevice(driver.getDevice());
        if (connection == null) return null;
        try {
            List<UsbSerialPort> ports = driver.getPorts();
            if (ports.isEmpty()) {
                connection.close();
                return null;
            }
            UsbSerialPort port = ports.get(0);
            openPort(port, connection);
            port.setDTR(true);
            Thread.sleep(200L);
            drain(port);
            return new UsbSerialServoBackend(UsbSerialServoBackend.BoardProtocol.SERVO2040,
                    port, null, pins, connection);
        } catch (Exception error) {
            try {
                connection.close();
            } catch (Exception ignored) {
            }
            return null;
        }
    }

    private static void openPort(UsbSerialPort port, UsbDeviceConnection connection) throws Exception {
        port.open(connection);
        port.setParameters(115200, 8, 1, UsbSerialPort.PARITY_NONE);
    }

    private static void drain(UsbSerialPort port) {
        byte[] buffer = new byte[1024];
        try {
            while (port.read(buffer, 100) > 0) {
                // Drain startup bytes, matching the decompiled Servo2040 backend.
            }
        } catch (Exception ignored) {
        }
    }

    private static void logDevices(UsbManager usbManager) {
        for (UsbDevice device : usbManager.getDeviceList().values()) {
            StringBuilder builder = new StringBuilder();
            builder.append("device name=").append(device.getDeviceName())
                    .append(" vid=").append(device.getVendorId())
                    .append(" pid=").append(device.getProductId())
                    .append(" class=").append(device.getDeviceClass())
                    .append(" subclass=").append(device.getDeviceSubclass())
                    .append(" protocol=").append(device.getDeviceProtocol())
                    .append(" interfaces=").append(device.getInterfaceCount());
            for (int i = 0; i < device.getInterfaceCount(); i++) {
                UsbInterface iface = device.getInterface(i);
                builder.append(" [").append(i)
                        .append(":class=").append(iface.getInterfaceClass())
                        .append(",subclass=").append(iface.getInterfaceSubclass())
                        .append(",protocol=").append(iface.getInterfaceProtocol())
                        .append(",endpoints=").append(iface.getEndpointCount())
                        .append("]");
            }
            Log.i(TAG, builder.toString());
        }
    }

    private static void logDrivers(String label, List<UsbSerialDriver> drivers) {
        Log.i(TAG, label + " drivers=" + drivers.size());
        for (UsbSerialDriver driver : drivers) {
            UsbDevice device = driver.getDevice();
            Log.i(TAG, label + " driver=" + driver.getClass().getSimpleName()
                    + " vid=" + device.getVendorId()
                    + " pid=" + device.getProductId()
                    + " ports=" + driver.getPorts().size());
        }
    }

    private static boolean isProbablyEmulator() {
        return Build.FINGERPRINT.contains("generic")
                || Build.FINGERPRINT.contains("emulator")
                || Build.FINGERPRINT.contains("sdk_gphone")
                || Build.HARDWARE.contains("ranchu")
                || Build.HARDWARE.contains("goldfish")
                || Build.DEVICE.contains("emu")
                || Build.MODEL.contains("sdk")
                || Build.MODEL.contains("Emulator")
                || Build.PRODUCT.contains("sdk");
    }
}
