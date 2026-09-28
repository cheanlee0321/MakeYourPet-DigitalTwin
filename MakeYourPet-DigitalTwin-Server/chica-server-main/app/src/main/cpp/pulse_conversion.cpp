#include "pulse_conversion.h"

#include <algorithm>

namespace {

// Defaults match the stock config-2040 (and the original z0.h initial values):
// every servo calibrated -45/+45 usec = 2000/1000, coxa attach -8 deg
// (-> {-8,0,8,-8,0,8}), femur 35, tibia 68, and the standard pin map.
ChicaServoConfig makeDefaultConfig()
{
    ChicaServoConfig c{};
    const int defaultPins[6][3] = {
        {15, 16, 17}, {9, 10, 11}, {3, 4, 5},
        {12, 13, 14}, {6, 7, 8}, {0, 1, 2},
    };
    const double coxa = -8.0;
    const double coxaExpanded[6] = {coxa, 0.0, -coxa, coxa, 0.0, -coxa};
    for (int leg = 0; leg < 6; ++leg) {
        c.coxaAttach[leg] = coxaExpanded[leg];
        for (int joint = 0; joint < 3; ++joint) {
            c.calibration[leg][joint][0] = 2000;  // -45 usec
            c.calibration[leg][joint][1] = 1000;  // +45 usec
            c.pin[leg][joint] = defaultPins[leg][joint];
        }
    }
    c.femurAttach = 35.0;
    c.tibiaAttach = 68.0;
    return c;
}

ChicaServoConfig g_config = makeDefaultConfig();

} // namespace

void chica_apk_set_servo_config(const ChicaServoConfig& config)
{
    g_config = config;
}

const ChicaServoConfig& chica_apk_servo_config()
{
    return g_config;
}

// Mirrors c2.n8.d(angle, leg, joint).
int chica_apk_angle_to_pulse(double angle_deg, int apk_leg_index, int joint_index)
{
    int leg = std::clamp(apk_leg_index, 0, 5);
    int joint = std::clamp(joint_index, 0, 2);
    bool right = leg > 2;

    double corrected = angle_deg;
    if (joint == 0) {
        corrected = (corrected - g_config.coxaAttach[leg]) * -1.0;
    } else if (joint == 1) {
        corrected -= g_config.femurAttach;
    } else {
        corrected = (corrected * -1.0) + g_config.tibiaAttach;
    }

    int low = g_config.calibration[leg][joint][0];   // -45 usec
    int high = g_config.calibration[leg][joint][1];   // +45 usec
    double scale = static_cast<double>(high - low) / 90.0;
    int center = (high + low) / 2;
    return (static_cast<int>(corrected * scale) * (right ? -1 : 1)) + center;
}

// Mirrors c2.n8.g(pulse, leg, joint).
double chica_apk_pulse_to_angle(int pulse, int apk_leg_index, int joint_index)
{
    int leg = std::clamp(apk_leg_index, 0, 5);
    int joint = std::clamp(joint_index, 0, 2);
    bool right = leg > 2;

    int low = g_config.calibration[leg][joint][0];
    int high = g_config.calibration[leg][joint][1];
    int center = (high + low) / 2;
    if (high == low) return 0.0;
    double scale = static_cast<double>(high - low) / 90.0;
    double corrected = (static_cast<double>(pulse - center) * (right ? -1.0 : 1.0)) / scale;

    if (joint == 0) return (corrected * -1.0) + g_config.coxaAttach[leg];
    if (joint == 1) return corrected + g_config.femurAttach;
    return (corrected - g_config.tibiaAttach) * -1.0;
}
