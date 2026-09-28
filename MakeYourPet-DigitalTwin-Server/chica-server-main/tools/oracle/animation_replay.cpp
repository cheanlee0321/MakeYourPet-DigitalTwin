#include "apk_model.h"
#include "pulse_conversion.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Engine {
    apk_model::RobotConfig config = apk_model::makeDefaultConfig();
    apk_model::WalkState state;
    std::array<apk_model::Pose, 4> layers = {};
    std::array<int, 18> lastPulses = {};
    std::array<bool, 6> activeOutput = {true, true, true, true, true, true};

    Engine()
    {
        apk_model::initializeWalkState(config, state);
    }
};

std::array<int, 18> toPulses(const std::array<std::array<double, 3>, 6>& angles)
{
    std::array<int, 18> pulses = {};
    for (int leg = 0; leg < 6; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            int pin = apk_model::ServoPinByApkLeg[leg][joint];
            pulses[pin] = chica_apk_angle_to_pulse(angles[leg][joint], leg, joint);
        }
    }
    return pulses;
}

apk_model::Pose addLayers(const Engine& engine)
{
    apk_model::Pose combined = {};
    for (const auto& layer : engine.layers) {
        combined = apk_model::addPose(combined, layer);
    }
    return combined;
}

std::array<int, 18> toPulsesFromPose(Engine& engine)
{
    std::array<std::array<double, 3>, 6> angles = {};
    std::array<bool, 6> active = {true, true, true, true, true, true};
    apk_model::Pose combined = addLayers(engine);
    engine.state.animation_layer = combined;
    if (!apk_model::inverseKinematics(engine.config, engine.state.body, combined, active, angles)) {
        return engine.lastPulses;
    }
    std::array<int, 18> next = toPulses(angles);
    for (int leg = 0; leg < 6; ++leg) {
        if (engine.activeOutput[leg]) continue;
        for (int pin : apk_model::ServoPinByApkLeg[leg]) {
            next[pin] = engine.lastPulses[pin];
        }
    }
    engine.lastPulses = next;
    return engine.lastPulses;
}

std::array<apk_model::Vec3, 6> neutralFeet(double radius,
                                           double z,
                                           double cornerAngleDeg,
                                           double elongation)
{
    return apk_model::makeNeutralFeet(
            radius,
            z,
            cornerAngleDeg,
            elongation,
            std::vector<int>(apk_model::ApkLegOrder.begin(), apk_model::ApkLegOrder.end()));
}

std::array<apk_model::Vec3, 6> neutralFeetForActive(double radius,
                                                    double z,
                                                    double cornerAngleDeg,
                                                    double elongation,
                                                    const std::vector<int>& active)
{
    return apk_model::makeNeutralFeet(radius, z, cornerAngleDeg, elongation, active);
}

apk_model::Pose lerpPose(const apk_model::Pose& from, const apk_model::Pose& to, double t)
{
    return {
        {
            from.xyz.x + ((to.xyz.x - from.xyz.x) * t),
            from.xyz.y + ((to.xyz.y - from.xyz.y) * t),
            from.xyz.z + ((to.xyz.z - from.xyz.z) * t),
        },
        {
            from.uvw.x + ((to.uvw.x - from.uvw.x) * t),
            from.uvw.y + ((to.uvw.y - from.uvw.y) * t),
            from.uvw.z + ((to.uvw.z - from.uvw.z) * t),
        },
    };
}

apk_model::Vec3 lerpVec(const apk_model::Vec3& from, const apk_model::Vec3& to, double t)
{
    return {
        from.x + ((to.x - from.x) * t),
        from.y + ((to.y - from.y) * t),
        from.z + ((to.z - from.z) * t),
    };
}

void scalePose(apk_model::Pose& pose, double amount)
{
    pose.xyz.x *= amount;
    pose.xyz.y *= amount;
    pose.xyz.z *= amount;
    pose.uvw.x *= amount;
    pose.uvw.y *= amount;
    pose.uvw.z *= amount;
}

apk_model::Pose originalAnimationPose(const Engine& engine, double phase, int animation)
{
    double angle = 3.14159265358979323846 * phase * 2.0;
    apk_model::Pose pose = {};
    switch (animation) {
        case 1:
            pose.xyz.z = (-std::cos(angle) * 60.0) + 30.0;
            pose.uvw.z = std::sin(angle) * 15.0;
            break;
        case 2:
            pose.xyz.x = -std::cos(angle) * 40.0;
            pose.uvw.y = std::sin(angle) * 15.0;
            break;
        case 3:
            pose.xyz.x = -std::cos(angle) * 40.0;
            pose.uvw.x = -std::sin(angle) * 15.0;
            break;
        case 4:
            pose.xyz.y = -std::cos(angle) * 60.0;
            pose.uvw.z = -std::sin(angle) * 15.0;
            break;
        case 5:
            pose.xyz.x = -std::cos(angle) * 40.0;
            pose.uvw.y = std::cos(angle) * 15.0;
            pose.uvw.z = -7.0;
            break;
        case 6:
            pose.xyz.x = -std::cos(angle) * 40.0;
            pose.xyz.y = std::sin(angle) * 50.0;
            break;
        case 7:
            pose.xyz.x = -std::cos(angle) * 40.0;
            pose.xyz.y = std::sin(angle) * 50.0;
            pose.uvw.y = std::cos(angle) * 12.0;
            pose.uvw.z = std::sin(angle) * 12.0;
            break;
        default: {
            double minZ = 1.7976931348623157E308;
            double maxZ = -1.7976931348623157E308;
            for (int leg : apk_model::ApkLegOrder) {
                minZ = std::min(minZ, engine.state.body.feet[leg].z);
                maxZ = std::max(maxZ, engine.state.body.feet[leg].z);
            }
            pose.xyz.z = (maxZ - minZ) / 2.0;
            break;
        }
    }
    scalePose(pose, 1.0);
    return pose;
}

void enterConstructorPose(Engine& engine)
{
    std::array<std::array<double, 3>, 6> angles = {};
    for (int leg = 0; leg < 6; ++leg) {
        angles[leg] = {0.0, 90.0, 120.0};
    }
    engine.state.initialized = true;
    engine.state.body = {};
    engine.state.animation_layer = {};
    engine.layers = {};
    engine.state.phase = 0.0;
    engine.state.anchors = {};
    engine.state.anchor_active = {};
    engine.activeOutput = {true, true, true, true, true, true};
    apk_model::forwardKinematics(engine.config, angles, engine.state.animation_layer, engine.state.body);
    engine.lastPulses = toPulses(angles);
}

void printFrame(const char* segment, double elapsed, const std::array<int, 18>& pulses)
{
    std::cout << "{\"type\":\"servo_values\",\"segment\":\"" << segment
              << "\",\"elapsed\":" << std::setprecision(17) << elapsed
              << ",\"values\":[";
    for (size_t i = 0; i < pulses.size(); ++i) {
        if (i > 0) std::cout << ",";
        std::cout << pulses[i];
    }
    std::cout << "]}\n";
}

void sampleBodyTarget(Engine& engine,
                      const char* segment,
                      const apk_model::BodyState& start,
                      const apk_model::BodyState& target,
                      double duration,
                      double elapsed)
{
    double t = std::min(1.0, std::max(0.0, elapsed) / std::max(1.0, duration));
    apk_model::BodyState next;
    next.body = lerpPose(start.body, target.body, t);
    for (int leg = 0; leg < 6; ++leg) {
        next.feet[leg] = lerpVec(start.feet[leg], target.feet[leg], t);
    }
    engine.state.body = next;
    printFrame(segment, elapsed, toPulsesFromPose(engine));
}

void finishTarget(Engine& engine, const apk_model::BodyState& target)
{
    engine.state.body = target;
    toPulsesFromPose(engine);
}

std::vector<int> parseLegs(const std::string& text)
{
    if (text == "all") {
        return {0, 3, 1, 4, 2, 5};
    }
    std::vector<int> legs;
    std::stringstream input(text);
    std::string part;
    while (std::getline(input, part, ',')) {
        if (!part.empty()) {
            legs.push_back(std::stoi(part));
        }
    }
    return legs;
}

bool runPose(Engine& engine, std::stringstream& line)
{
    double threshold = 0.0;
    double lift = 0.0;
    double layerBlend = 0.0;
    double duration = 0.0;
    std::string legsText;
    if (!(line >> threshold >> lift >> layerBlend >> duration >> legsText)) {
        return false;
    }
    std::vector<int> requested = parseLegs(legsText);
    apk_model::BodyState start = engine.state.body;
    apk_model::BodyState target = start;
    std::vector<int> moving;
    for (int leg : requested) {
        apk_model::Vec3 neutral = apk_model::neutralFootForBody(engine.config, start, {}, leg);
        target.feet[leg] = neutral;
        double dx = neutral.x - start.feet[leg].x;
        double dy = neutral.y - start.feet[leg].y;
        if (threshold < 0.0 || ((dx * dx) + (dy * dy)) > (threshold * threshold)) {
            moving.push_back(leg);
        }
    }

    double elapsed = 0.0;
    while (line >> elapsed) {
        double t = std::min(1.0, std::max(0.0, elapsed) / std::max(1.0, duration));
        apk_model::BodyState next = start;
        for (int leg : moving) {
            next.feet[leg] = apk_model::swingTrajectory(start.feet[leg], target.feet[leg], t, lift);
        }
        if (layerBlend > 0.0) {
            apk_model::Pose layerTarget = originalAnimationPose(engine, t, 0);
            engine.layers[1] = lerpPose(engine.layers[1], layerTarget, layerBlend);
        }
        engine.state.body = next;
        printFrame("pose", elapsed, toPulsesFromPose(engine));
    }
    engine.state.body = target;
    toPulsesFromPose(engine);
    return true;
}

bool runShape(Engine& engine, std::stringstream& line)
{
    double radius = 0.0;
    double z = 0.0;
    double corner = 0.0;
    double elongation = 0.0;
    double duration = 0.0;
    if (!(line >> radius >> z >> corner >> elongation >> duration)) {
        return false;
    }
    apk_model::BodyState start = engine.state.body;
    apk_model::BodyState target = start;
    auto feet = neutralFeet(radius, z, corner, elongation);
    for (int leg : apk_model::ApkLegOrder) {
        target.feet[leg] = feet[leg];
    }

    double elapsed = 0.0;
    while (line >> elapsed) {
        sampleBodyTarget(engine, "shape", start, target, duration, elapsed);
    }
    finishTarget(engine, target);
    return true;
}

bool runShapeLegs(Engine& engine, std::stringstream& line)
{
    std::string legsText;
    double radius = 0.0;
    double z = 0.0;
    double corner = 0.0;
    double elongation = 0.0;
    double duration = 0.0;
    if (!(line >> legsText >> radius >> z >> corner >> elongation >> duration)) {
        return false;
    }
    std::vector<int> legs = parseLegs(legsText);
    apk_model::BodyState start = engine.state.body;
    apk_model::BodyState target = start;
    auto feet = neutralFeet(radius, z, corner, elongation);
    for (int leg : legs) {
        if (leg < 0 || leg >= 6) continue;
        target.feet[leg] = feet[leg];
    }

    double elapsed = 0.0;
    while (line >> elapsed) {
        sampleBodyTarget(engine, "shapelegs", start, target, duration, elapsed);
    }
    finishTarget(engine, target);
    return true;
}

bool runConfig(Engine& engine, std::stringstream& line)
{
    double radius = 0.0;
    double z = 0.0;
    double corner = 0.0;
    double elongation = 0.0;
    if (!(line >> radius >> z >> corner >> elongation)) {
        return false;
    }
    engine.config.leg_radius = radius;
    engine.config.leg_sitting_z = z;
    engine.config.corner_leg_angle_deg = corner;
    engine.config.elongation = elongation;
    engine.config.neutral_feet = neutralFeet(radius, z, corner, elongation);
    return true;
}

bool runConfigQuadPair(Engine& engine, std::stringstream& line)
{
    std::string pairText;
    if (!(line >> pairText)) {
        return false;
    }
    std::vector<int> active;
    if (pairText == "0,3") {
        active = {5, 2, 1, 4};
    } else if (pairText == "1,4") {
        active = {5, 2, 0, 3};
    } else if (pairText == "2,5") {
        active = {1, 0, 3, 4};
    } else {
        return false;
    }
    engine.config.leg_radius = 220.0;
    engine.config.leg_sitting_z = -40.0;
    engine.config.corner_leg_angle_deg = 60.0;
    engine.config.elongation = 1.0;
    engine.config.neutral_feet = neutralFeetForActive(220.0, -40.0, 60.0, 1.0, active);
    return true;
}

bool runActive(Engine& engine, std::stringstream& line)
{
    std::string legsText;
    if (!(line >> legsText)) {
        return false;
    }
    engine.activeOutput = {false, false, false, false, false, false};
    for (int leg : parseLegs(legsText)) {
        if (leg >= 0 && leg < 6) {
            engine.activeOutput[leg] = true;
        }
    }
    return true;
}

bool runBodyZ(Engine& engine, std::stringstream& line)
{
    double bodyZ = 0.0;
    double duration = 0.0;
    if (!(line >> bodyZ >> duration)) {
        return false;
    }
    apk_model::BodyState start = engine.state.body;
    apk_model::BodyState target = start;
    target.body.xyz.z = bodyZ;

    double elapsed = 0.0;
    while (line >> elapsed) {
        sampleBodyTarget(engine, "bodyz", start, target, duration, elapsed);
    }
    finishTarget(engine, target);
    return true;
}

bool runBodyZDelta(Engine& engine, std::stringstream& line)
{
    double bodyZDelta = 0.0;
    double duration = 0.0;
    if (!(line >> bodyZDelta >> duration)) {
        return false;
    }
    apk_model::BodyState start = engine.state.body;
    apk_model::BodyState target = start;
    target.body.xyz.z += bodyZDelta;

    double elapsed = 0.0;
    while (line >> elapsed) {
        sampleBodyTarget(engine, "bodyzdelta", start, target, duration, elapsed);
    }
    finishTarget(engine, target);
    return true;
}

} // namespace

int main()
{
    Engine engine;
    std::string raw;
    while (std::getline(std::cin, raw)) {
        if (raw.empty() || raw[0] == '#') {
            continue;
        }
        std::stringstream line(raw);
        std::string command;
        line >> command;
        if (command == "constructor") {
            enterConstructorPose(engine);
        } else if (command == "shape") {
            if (!runShape(engine, line)) {
                std::cerr << "invalid shape command: " << raw << "\n";
                return 2;
            }
        } else if (command == "shapelegs") {
            if (!runShapeLegs(engine, line)) {
                std::cerr << "invalid shapelegs command: " << raw << "\n";
                return 2;
            }
        } else if (command == "config") {
            if (!runConfig(engine, line)) {
                std::cerr << "invalid config command: " << raw << "\n";
                return 2;
            }
        } else if (command == "configquadpair") {
            if (!runConfigQuadPair(engine, line)) {
                std::cerr << "invalid configquadpair command: " << raw << "\n";
                return 2;
            }
        } else if (command == "active") {
            if (!runActive(engine, line)) {
                std::cerr << "invalid active command: " << raw << "\n";
                return 2;
            }
        } else if (command == "bodyz") {
            if (!runBodyZ(engine, line)) {
                std::cerr << "invalid bodyz command: " << raw << "\n";
                return 2;
            }
        } else if (command == "bodyzdelta") {
            if (!runBodyZDelta(engine, line)) {
                std::cerr << "invalid bodyzdelta command: " << raw << "\n";
                return 2;
            }
        } else if (command == "pose") {
            if (!runPose(engine, line)) {
                std::cerr << "invalid pose command: " << raw << "\n";
                return 2;
            }
        } else {
            std::cerr << "unknown command: " << command << "\n";
            return 2;
        }
    }
    return 0;
}
