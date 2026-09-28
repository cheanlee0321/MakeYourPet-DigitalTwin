package com.makeyourpet.chicaserver.control;

import android.content.Context;
import android.util.Log;
import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtException;
import ai.onnxruntime.OrtSession;
import com.makeyourpet.chicaserver.hardware.ChicaServoCalibration;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Collections;

/**
 * OnnxLocomotionRunner
 * 
 * MakeYourPet Hexapod ONNX Residual Locomotion Inference Engine.
 * 1. Computes geometric tripod kinematics feedforward trajectory q_ref.
 * 2. Samples phone IMU and robot states in real-time to construct the 67-dim observation vector matching MuJoCo training spec.
 * 3. Runs neural network inference (hexapod_policy.onnx) via ONNX Runtime Mobile to output 18-dim residual actions.
 * 4. Scales and blends residuals (q = q_ref + 0.15 * delta_q), applying mechanical joint limits and EMA filtering.
 * 5. Provides accurate mapping from 18 joint angles (rad) to Pimoroni Servo 2040 physical channel pulse widths (usec).
 */
public final class OnnxLocomotionRunner implements AutoCloseable {
    private static final String TAG = "OnnxLocomotionRunner";

    public static final int OBS_DIM = 67;
    public static final int ACTION_DIM = 18;
    public static final float RESIDUAL_SCALE = 0.15f;
    public static final float EMA_BETA = 0.7f;
    public static final double STEP_FREQUENCY = 1.5; // 1.5 Hz stepping frequency
    public static final float DEFAULT_DT = 0.02f;   // 20ms (50Hz)

    /**
     * Mapping from 18 hexapod joints to Pimoroni Servo 2040 physical pins (Channels 0~17):
     * L1: Coxa=15, Femur=16, Tibia=17
     * L2: Coxa=9,  Femur=10, Tibia=11
     * L3: Coxa=3,  Femur=4,  Tibia=5
     * R1: Coxa=12, Femur=13, Tibia=14
     * R2: Coxa=6,  Femur=7,  Tibia=8
     * R3: Coxa=0,  Femur=1,  Tibia=2
     */
    public static final int[] JOINT_TO_PIN_MAP = {
            15, 16, 17,  // Leg 0 (L1): Joint 0, 1, 2
            9,  10, 11,  // Leg 1 (L2): Joint 3, 4, 5
            3,  4,  5,   // Leg 2 (L3): Joint 6, 7, 8
            12, 13, 14,  // Leg 3 (R1): Joint 9, 10, 11
            6,  7,  8,   // Leg 4 (R2): Joint 12, 13, 14
            0,  1,  2    // Leg 5 (R3): Joint 15, 16, 17
    };

    /**
     * Default neutral standing pulse widths in microseconds (Pins 0~17):
     * 0, 1, 2:   R3 (1627, 1596, 1509)
     * 3, 4, 5:   L3 (1373, 1404, 1491)
     * 6, 7, 8:   R2 (1500, 1625, 1575)
     * 9, 10, 11: L2 (1500, 1375, 1425)
     * 12, 13, 14: R1 (1373, 1596, 1509)
     * 15, 16, 17: L1 (1627, 1404, 1491)
     */
    public static final int[] DEFAULT_STAND_PULSES = {
            1627, 1596, 1509,
            1373, 1404, 1491,
            1500, 1625, 1575,
            1500, 1375, 1425,
            1373, 1596, 1509,
            1627, 1404, 1491
    };

    // Mechanical attachment angles (degrees)
    private static final double[] DEFAULT_COXA_ATTACH = {-8.0, 0.0, 8.0, -8.0, 0.0, 8.0};
    private static final double DEFAULT_FEMUR_ATTACH = 35.0;
    private static final double DEFAULT_TIBIA_ATTACH = 68.0;

    // Joint limits (rad)
    private static final float COXA_LIMIT = 0.7853982f;  // +/- 45 deg
    private static final float FEMUR_LIMIT = 0.7853982f; // +/- 45 deg
    private static final float TIBIA_LIMIT = 1.0471976f; // +/- 60 deg

    // Hexapod leg configuration: (isTripodA, isRight, yawMultiplier)
    // 0: L1 (A, Left),  1: L2 (B, Left),  2: L3 (A, Left)
    // 3: R1 (B, Right), 4: R2 (A, Right), 5: R3 (B, Right)
    private static final boolean[] LEG_IS_TRIPOD_A = {true, false, true, false, true, false};
    private static final boolean[] LEG_IS_RIGHT = {false, false, false, true, true, true};
    private static final float[] LEG_YAW_MULT = {-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f};

    private final OrtEnvironment env;
    private final OrtSession session;
    private final String inputTensorName;
    private final String outputTensorName;

    // Kinematic and posture parameters
    private float liftFemur = 0.32f;
    private float liftTibia = 0.20f;
    private float scaleFront = 1.0f;
    private float scaleMiddle = 1.0f;
    private float scaleRear = 1.30f;
    private float strideGain = 2.0f;
    private float yawGain = 0.40f;

    // Historical state buffers
    private final float[] prevAction = new float[ACTION_DIM];
    private final float[] currentJointAngles = new float[ACTION_DIM];
    private final float[] prevJointAngles = new float[ACTION_DIM];
    private final float[] smoothedTargetAngles = new float[ACTION_DIM];
    private double gaitPhase = 0.0;
    private boolean initialized = false;

    // Preallocated inference buffers (Zero-Allocation on 50Hz hot path)
    private final float[] obsBuffer = new float[OBS_DIM];
    private final float[] residualBuffer = new float[ACTION_DIM];
    private final FloatBuffer inputFloatBuffer = ByteBuffer.allocateDirect(OBS_DIM * 4)
            .order(ByteOrder.nativeOrder())
            .asFloatBuffer();
    private final long[] inputShape = new long[]{1, OBS_DIM};
    private final int[] channelPulsesBuffer = new int[18];

    // Calibration parameters (optional custom calibration)
    private ChicaServoCalibration servoCalibration;

    /**
     * Load ONNX model from Android Assets.
     */
    public OnnxLocomotionRunner(Context context, String assetPath) throws Exception {
        this(loadAssetBytes(context, assetPath));
    }

    /**
     * Initialize ONNX runtime environment directly from byte[].
     */
    public OnnxLocomotionRunner(byte[] modelBytes) throws Exception {
        this.env = OrtEnvironment.getEnvironment();
        OrtSession.SessionOptions opts = new OrtSession.SessionOptions();
        opts.setIntraOpNumThreads(2);
        this.session = env.createSession(modelBytes, opts);

        // Dynamically detect model input and output tensor names
        String inName = "observation";
        if (!session.getInputNames().isEmpty()) {
            inName = session.getInputNames().iterator().next();
        }
        this.inputTensorName = inName;

        String outName = "action";
        if (!session.getOutputNames().isEmpty()) {
            outName = session.getOutputNames().iterator().next();
        }
        this.outputTensorName = outName;

        reset();
        Log.i(TAG, "ONNX Locomotion Policy loaded successfully. Input: " + inputTensorName + ", Output: " + outputTensorName);
    }

    public void setServoCalibration(ChicaServoCalibration calibration) {
        this.servoCalibration = calibration;
    }

    public void setHighClearance(boolean enabled) {
        if (enabled) {
            this.liftFemur = 0.55f;
            this.liftTibia = 0.36f;
        } else {
            this.liftFemur = 0.32f;
            this.liftTibia = 0.20f;
        }
    }

    /**
     * Reset all action buffers, historical angles, and gait phase clock.
     */
    public synchronized void reset() {
        for (int i = 0; i < ACTION_DIM; i++) {
            prevAction[i] = 0.0f;
            currentJointAngles[i] = 0.0f;
            prevJointAngles[i] = 0.0f;
            smoothedTargetAngles[i] = 0.0f;
        }
        gaitPhase = 0.0;
        initialized = true;
    }

    /**
     * Execute one complete gait inference cycle (default 50Hz, 20ms).
     * 
     * @param vx Target forward velocity (m/s)
     * @param vy Target lateral velocity (m/s)
     * @param yawRate Target yaw angular velocity (rad/s)
     * @param roll Body roll angle (rad)
     * @param pitch Body pitch angle (rad)
     * @param omega Body angular velocity [wx, wy, wz] (rad/s)
     * @param dt Cycle time step (seconds, e.g. 0.02)
     * @return 18-dim blended and smoothed target joint angles (rad)
     */
    public synchronized float[] step(float vx, float vy, float yawRate,
                                     float roll, float pitch, float[] omega,
                                     float dt) throws OrtException {
        if (dt <= 0.0f) dt = DEFAULT_DT;

        // 1. Advance gait clock
        boolean isMoving = Math.abs(vx) > 0.02f || Math.abs(yawRate) > 0.05f;
        if (isMoving) {
            gaitPhase = (gaitPhase + 2.0 * Math.PI * STEP_FREQUENCY * dt) % (2.0 * Math.PI);
        } else {
            gaitPhase = 0.0;
        }

        // 2. Compute geometric tripod kinematics feedforward reference angles q_ref (rad)
        float[] qRef = computeReferenceAngles(gaitPhase, vx, vy, yawRate);

        // 3. Assemble 67-dim observation vector (strictly aligned with MuJoCo hexapod_env.py)
        float[] obs = buildObservation(vx, vy, yawRate, roll, pitch, omega, qRef, dt);

        // 4. ONNX Runtime inference for action residual delta_q (smoothly decays to 0 when stationary)
        float[] residual;
        if (isMoving) {
            residual = inferResidual(obs);
            System.arraycopy(residual, 0, prevAction, 0, ACTION_DIM);
        } else {
            for (int i = 0; i < ACTION_DIM; i++) {
                prevAction[i] *= 0.85f;
            }
            residual = prevAction;
        }

        // 5. Update previous joint angles
        System.arraycopy(currentJointAngles, 0, prevJointAngles, 0, ACTION_DIM);

        // 6. Residual superposition, joint limit clipping, and EMA smoothing
        float beta = isMoving ? EMA_BETA : 0.4f;
        for (int i = 0; i < ACTION_DIM; i++) {
            float target = qRef[i] + residual[i] * RESIDUAL_SCALE;

            // Software mechanical joint limits
            int jointInLeg = i % 3;
            float limit = (jointInLeg == 2) ? TIBIA_LIMIT : (jointInLeg == 1 ? FEMUR_LIMIT : COXA_LIMIT);
            target = Math.max(-limit, Math.min(limit, target));

            // EMA (Exponential Moving Average) smoothing (gentle deceleration to neutral when stopping)
            smoothedTargetAngles[i] = (1.0f - beta) * smoothedTargetAngles[i] + beta * target;
            currentJointAngles[i] = smoothedTargetAngles[i];
        }

        return smoothedTargetAngles.clone();
    }

    /**
     * Execute inference and convert directly to Servo 2040 physical channel pulse widths (18-element array).
     */
    public synchronized int[] stepPulses(float vx, float vy, float yawRate,
                                         float roll, float pitch, float[] omega,
                                         float dt) throws OrtException {
        float[] anglesRad = step(vx, vy, yawRate, roll, pitch, omega, dt);
        return anglesToServoPulses(anglesRad);
    }

    /**
     * Compute analytical tripod gait feedforward reference angles q_ref (rad).
     * Exact mirror of python tripod_kinematics.py.
     */
    public float[] computeReferenceAngles(double phase, float vx, float vy, float yaw) {
        float[] qRef = new float[ACTION_DIM];
        float speed = Math.abs(vx) + Math.abs(vy) + Math.abs(yaw);

        // Stationary stance
        if (speed < 0.02f) {
            return qRef;
        }

        double phaseA = phase % (2.0 * Math.PI);
        double phaseB = (phase + Math.PI) % (2.0 * Math.PI);
        float strideBase = vx * strideGain;

        for (int leg = 0; leg < 6; leg++) {
            boolean isTripodA = LEG_IS_TRIPOD_A[leg];
            boolean isRight = LEG_IS_RIGHT[leg];
            float yawMult = LEG_YAW_MULT[leg];

            double p = isTripodA ? phaseA : phaseB;
            float legStride = Math.max(-0.65f, Math.min(0.65f, strideBase + yaw * yawGain * yawMult));

            // 1. Coxa yaw angle
            float qCoxa = (isRight ? -1.0f : 1.0f) * (float) Math.cos(p) * legStride;

            // 2. Femur & Tibia elevation angles (swing phase vs. stance phase)
            float qFemur;
            float qTibia;
            if (p < Math.PI) {
                // Swing phase: high leg lift (sin(p)^0.8)
                float h = (float) Math.pow(Math.sin(p), 0.8);
                float liftMult = (leg == 1 || leg == 4) ? scaleMiddle : ((leg == 0 || leg == 3) ? scaleFront : scaleRear);
                qFemur = -liftFemur * liftMult * h;
                qTibia = -liftTibia * liftMult * h;
            } else {
                // Stance phase: ground support
                qFemur = 0.01f;
                qTibia = 0.005f;
            }

            int baseJ = leg * 3;
            qRef[baseJ + 0] = qCoxa;
            qRef[baseJ + 1] = qFemur;
            qRef[baseJ + 2] = qTibia;
        }

        return qRef;
    }

    /**
     * Assemble 67-dim observation state vector:
     * [0:2]   roll, pitch (rad)
     * [2:5]   omega [wx, wy, wz] (rad/s)
     * [5:8]   v_body [vx, vy, vz] (m/s)
     * [8:26]  joint_error (current - qRef) (18)
     * [26:44] joint_vel * 0.1 (18)
     * [44:62] prev_action (18)
     * [62:65] command [vx, vy, yaw] (3)
     * [65:67] clock [sin(phase), cos(phase)] (2)
     */
    public float[] buildObservation(float vx, float vy, float yaw,
                                    float roll, float pitch, float[] omega,
                                    float[] qRef, float dt) {
        // 1. Body orientation (2)
        obsBuffer[0] = roll;
        obsBuffer[1] = pitch;

        // 2. Body angular velocity (3)
        if (omega != null && omega.length >= 3) {
            obsBuffer[2] = omega[0];
            obsBuffer[3] = omega[1];
            obsBuffer[4] = omega[2];
        } else {
            obsBuffer[2] = 0.0f;
            obsBuffer[3] = 0.0f;
            obsBuffer[4] = 0.0f;
        }

        // 3. Body linear velocity (3)
        obsBuffer[5] = vx;
        obsBuffer[6] = vy;
        obsBuffer[7] = 0.0f;

        // 4. Joint tracking error (18)
        for (int i = 0; i < ACTION_DIM; i++) {
            obsBuffer[8 + i] = currentJointAngles[i] - qRef[i];
        }

        // 5. Scaled joint velocity (18)
        float invDt = dt > 1e-4f ? (1.0f / dt) : 50.0f;
        for (int i = 0; i < ACTION_DIM; i++) {
            float dq = (currentJointAngles[i] - prevJointAngles[i]) * invDt;
            obsBuffer[26 + i] = dq * 0.1f;
        }

        // 6. Previous residual action (18)
        System.arraycopy(prevAction, 0, obsBuffer, 44, ACTION_DIM);

        // 7. Target velocity command (3)
        obsBuffer[62] = vx;
        obsBuffer[63] = vy;
        obsBuffer[64] = yaw;

        // 8. Gait phase clock (2)
        boolean isMoving = Math.abs(vx) > 0.02f || Math.abs(yaw) > 0.05f;
        obsBuffer[65] = isMoving ? (float) Math.sin(gaitPhase) : 0.0f;
        obsBuffer[66] = isMoving ? (float) Math.cos(gaitPhase) : 0.0f;

        return obsBuffer;
    }

    /**
     * Invoke ONNX Runtime for single-step inference (reusing zero-allocation buffers).
     */
    private float[] inferResidual(float[] obs) throws OrtException {
        inputFloatBuffer.clear();
        inputFloatBuffer.put(obs);
        inputFloatBuffer.flip();

        try (OnnxTensor inputTensor = OnnxTensor.createTensor(env, inputFloatBuffer, inputShape);
             OrtSession.Result result = session.run(Collections.singletonMap(inputTensorName, inputTensor))) {

            Object val = result.get(0).getValue();
            if (val instanceof float[][]) {
                float[][] raw = (float[][]) val;
                for (int i = 0; i < ACTION_DIM && i < raw[0].length; i++) {
                    residualBuffer[i] = Math.max(-1.0f, Math.min(1.0f, raw[0][i]));
                }
            } else if (val instanceof float[]) {
                float[] raw = (float[]) val;
                for (int i = 0; i < ACTION_DIM && i < raw.length; i++) {
                    residualBuffer[i] = Math.max(-1.0f, Math.min(1.0f, raw[i]));
                }
            }
            return residualBuffer;
        }
    }

    /**
     * Convert 18 joint angles (rad) to Pimoroni Servo 2040 physical channel pulse widths (usec).
     * Output array length is 18, indexed by board Channels 0~17.
     *
     * Geometric coordinate alignment:
     * MuJoCo / ONNX model output represents relative offsets around standard standing pose (0.0 rad = Stand Pose).
     * 1. Baseline pulse widths use standard neutral standing pulses DEFAULT_STAND_PULSES (or custom calibration neutral).
     * 2. Coxa (Yaw): Negative angle swings leg forward on Left, positive angle swings leg forward on Right.
     * 3. Femur (Pitch): Negative angle lifts leg (Left pulse width decreases, Right pulse width increases).
     * 4. Tibia (Pitch): Negative angle flexes knee (Left pulse width decreases, Right pulse width increases).
     */
    public int[] anglesToServoPulses(float[] jointAnglesRad) {
        // Fill default standard standing pulse widths
        System.arraycopy(DEFAULT_STAND_PULSES, 0, channelPulsesBuffer, 0, 18);

        for (int jointIdx = 0; jointIdx < ACTION_DIM; jointIdx++) {
            int leg = jointIdx / 3;
            int joint = jointIdx % 3;
            boolean right = leg > 2;
            float sideMult = right ? -1.0f : 1.0f;

            // Get physical pin (prioritize custom calibration settings)
            int pin = (servoCalibration != null && servoCalibration.pin != null
                    && leg < servoCalibration.pin.length && joint < servoCalibration.pin[leg].length)
                    ? servoCalibration.pin[leg][joint]
                    : JOINT_TO_PIN_MAP[jointIdx];

            if (pin < 0 || pin >= 18) continue;

            float angleDeg = (float) Math.toDegrees(jointAnglesRad[jointIdx]);

            // Compute neutral center pulse width for each joint (usec)
            int center = DEFAULT_STAND_PULSES[pin];
            float degScale = 11.1111f; // Default 90 deg maps to 1000 usec (11.111 usec/deg)

            if (servoCalibration != null
                    && leg < servoCalibration.calibration.length
                    && joint < servoCalibration.calibration[leg].length) {
                int low = servoCalibration.calibration[leg][joint][0];
                int high = servoCalibration.calibration[leg][joint][1];
                degScale = Math.abs((float) (high - low)) / 90.0f;
                center = (high + low) / 2;
                if (joint == 0 && servoCalibration.coxaAttach != null && leg < servoCalibration.coxaAttach.length) {
                    center += (int) (-servoCalibration.coxaAttach[leg] * degScale);
                }
            }

            float deltaUsec;
            if (joint == 0) {
                // Coxa: -angleDeg * degScale automatically accommodates bilateral swing forward
                deltaUsec = -angleDeg * degScale;
            } else {
                // Femur & Tibia: sideMult compensates for mirrored bilateral mounting
                deltaUsec = sideMult * angleDeg * degScale;
            }

            int pulse = Math.round(center + deltaUsec);
            // Physical mechanical safety limit [700, 2300] usec
            channelPulsesBuffer[pin] = Math.max(700, Math.min(2300, pulse));
        }

        return channelPulsesBuffer.clone();
    }

    public double getGaitPhase() {
        return gaitPhase;
    }

    public float[] getSmoothedTargetAngles() {
        return smoothedTargetAngles.clone();
    }

    public float[] getPrevAction() {
        return prevAction.clone();
    }

    @Override
    public synchronized void close() {
        try {
            if (session != null) session.close();
        } catch (Exception ignored) {
        }
        try {
            if (env != null) env.close();
        } catch (Exception ignored) {
        }
    }

    private static byte[] loadAssetBytes(Context context, String assetPath) throws Exception {
        try (InputStream is = context.getAssets().open(assetPath);
             ByteArrayOutputStream baos = new ByteArrayOutputStream()) {
            byte[] buf = new byte[4096];
            int n;
            while ((n = is.read(buf)) != -1) {
                baos.write(buf, 0, n);
            }
            return baos.toByteArray();
        }
    }
}
