package com.makeyourpet.chicaserver;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import com.makeyourpet.chicaserver.control.ChicaController;

public final class OriginalOrientationSensor implements SensorEventListener {
    private final ChicaController controller;
    private final SensorManager sensorManager;
    private final Sensor gravitySensor;
    private final Sensor magneticSensor;
    private final float[] rotation = new float[9];
    private final float[] inclination = new float[9];
    private final float[] orientation = new float[3];
    private float[] gravity;
    private float[] magnetic;
    private boolean hasGravity = false;
    private boolean hasMagnetic = false;
    private boolean registered = false;

    public OriginalOrientationSensor(Context context, ChicaController controller) {
        this.controller = controller;
        sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        Sensor gSensor = sensorManager == null ? null : sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        if (gSensor == null && sensorManager != null) {
            gSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        }
        gravitySensor = gSensor;
        magneticSensor = sensorManager == null ? null : sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
    }

    public void start() {
        if (registered || sensorManager == null || gravitySensor == null || magneticSensor == null) return;
        sensorManager.registerListener(this, gravitySensor, SensorManager.SENSOR_DELAY_GAME, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(this, magneticSensor, SensorManager.SENSOR_DELAY_GAME, SensorManager.SENSOR_DELAY_GAME);
        registered = true;
    }

    public void stop() {
        if (!registered || sensorManager == null) return;
        sensorManager.unregisterListener(this);
        registered = false;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        int type = event.sensor.getType();
        if (type == Sensor.TYPE_GRAVITY || type == Sensor.TYPE_ACCELEROMETER) {
            if (gravity == null || gravity.length != event.values.length) {
                gravity = new float[event.values.length];
            }
            System.arraycopy(event.values, 0, gravity, 0, gravity.length);
            hasGravity = true;
        } else if (type == Sensor.TYPE_MAGNETIC_FIELD) {
            if (magnetic == null || magnetic.length != event.values.length) {
                magnetic = new float[event.values.length];
            }
            for (int i = 0; i < magnetic.length; i++) {
                // Match the original's lerp operation order exactly (e2.w8.i:
                // (1-t)*a + b*t), not the algebraically-equal a+(b-a)*t -- the
                // two round differently and this feeds a recursive filter.
                magnetic[i] = (float) (((1.0d - 0.2d) * magnetic[i]) + (event.values[i] * 0.2d));
            }
            hasMagnetic = true;
        }
        if (hasGravity && hasMagnetic) {
            if (SensorManager.getRotationMatrix(rotation, inclination, gravity, magnetic)) {
                SensorManager.getOrientation(rotation, orientation);
                controller.setOrientationVector(-orientation[2], orientation[1], -orientation[0]);
            }
            hasGravity = false;
            hasMagnetic = false;
        }
    }
}
