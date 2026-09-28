#pragma once

// Per-servo calibration + mechanical attach angles + servo pin map, mirroring
// the original config (z0.h fields populated by a2.n.f) used in the angle->pulse
// conversion c2.n8.d. Defaults match the stock config-2040 so an unconfigured
// engine produces byte-identical pulses to before.
struct ChicaServoConfig {
    int calibration[6][3][2];  // [apk_leg][joint][0]=-45 usec, [1]=+45 usec
    double coxaAttach[6];      // COXA_ATTACH_ANGLE expanded: {d,0,-d,d,0,-d}
    double femurAttach;        // FEMUR_ATTACH_ANGLE
    double tibiaAttach;        // TIBIA_ATTACH_ANGLE
    int pin[6][3];             // [apk_leg][joint] -> board pin (servo config column)
};

// Set the active servo config (calibration/attach/pin). Pass nullptr fields are
// not supported; callers fill the whole struct (Java parses the config).
void chica_apk_set_servo_config(const ChicaServoConfig& config);

// Current config (defaults until set), exposed for the pin map in toPulses.
const ChicaServoConfig& chica_apk_servo_config();

int chica_apk_angle_to_pulse(double angle_deg, int apk_leg_index, int joint_index);
double chica_apk_pulse_to_angle(int pulse, int apk_leg_index, int joint_index);
