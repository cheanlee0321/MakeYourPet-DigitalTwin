#include "apk_model.h"
#include "pulse_conversion.h"

#include <array>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr std::array<int, 18> OriginalStandPulses = {
    1627, 1596, 1509,
    1373, 1404, 1491,
    1500, 1625, 1575,
    1500, 1375, 1425,
    1373, 1596, 1509,
    1627, 1404, 1491,
};

std::array<int, 18> toPulses(const apk_model::WalkStepResult& step)
{
    std::array<int, 18> pulses = {};
    for (int leg = 0; leg < 6; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            int pin = apk_model::ServoPinByApkLeg[leg][joint];
            pulses[pin] = chica_apk_angle_to_pulse(step.angles_deg[leg][joint], leg, joint);
        }
    }
    return pulses;
}

bool parsePulses(const std::string& text, std::array<int, 18>& pulses)
{
    std::stringstream input(text);
    std::string part;
    std::vector<std::string> parts;
    while (std::getline(input, part, ',')) {
        parts.push_back(part);
    }
    if (parts.size() != 18) return false;
    for (size_t i = 0; i < pulses.size(); ++i) {
        pulses[i] = std::stoi(parts[i]);
    }
    return true;
}

std::vector<int> parseLegs(const std::string& text)
{
    std::stringstream input(text);
    std::string part;
    std::vector<int> legs;
    while (std::getline(input, part, ',')) {
        if (!part.empty()) legs.push_back(std::stoi(part));
    }
    return legs;
}

void seedFromPulses(const apk_model::RobotConfig& config,
                    apk_model::WalkState& state,
                    const std::array<int, 18>& pulses,
                    double body_z)
{
    apk_model::initializeWalkState(config, state);
    state.body.body.xyz = {0.0, 0.0, body_z};
    state.body.body.uvw = {0.0, 0.0, 0.0};
    state.animation_layer = {};
    state.phase = 0.0;
    state.anchors = {};
    state.anchor_active = {};

    std::array<std::array<double, 3>, 6> angles = {};
    for (int leg = 0; leg < 6; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            int pin = apk_model::ServoPinByApkLeg[leg][joint];
            angles[leg][joint] = chica_apk_pulse_to_angle(pulses[pin], leg, joint);
        }
    }
    apk_model::forwardKinematics(config, angles, state.animation_layer, state.body);
}

void printPulses(const std::array<int, 18>& pulses)
{
    std::cout << "[";
    for (size_t i = 0; i < pulses.size(); ++i) {
        if (i != 0) std::cout << ",";
        std::cout << pulses[i];
    }
    std::cout << "]";
}

struct ProbeFrame {
    int gait = 5;
    int style = 0;
    double dt = 0.0;
    bool allow = true;
    apk_model::WalkCommand command;
};

bool parseFrame(const std::string& text, ProbeFrame& frame)
{
    std::stringstream input(text);
    std::string part;
    std::vector<std::string> parts;
    while (std::getline(input, part, ',')) {
        parts.push_back(part);
    }
    if (parts.size() != 7) return false;
    frame.gait = std::stoi(parts[0]);
    frame.style = std::stoi(parts[1]);
    frame.dt = std::stod(parts[2]);
    frame.allow = std::stoi(parts[3]) != 0;
    frame.command.forward = std::stod(parts[4]);
    frame.command.left = std::stod(parts[5]);
    frame.command.turn = std::stod(parts[6]);
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    apk_model::RobotConfig config = apk_model::makeDefaultConfig();
    apk_model::WalkState state;
    bool pulse_seed = false;
    double body_z = 40.0;
    std::array<int, 18> seed_pulses = OriginalStandPulses;
    std::array<bool, 6> active = {true, true, true, true, true, true};
    std::vector<ProbeFrame> frames;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--pulse-seed") {
            pulse_seed = true;
        } else if (arg == "--seed-pulses" && i + 1 < argc) {
            if (!parsePulses(argv[++i], seed_pulses)) {
                std::cerr << "invalid --seed-pulses value\n";
                return 2;
            }
            pulse_seed = true;
        } else if (arg == "--body-z" && i + 1 < argc) {
            body_z = std::stod(argv[++i]);
        } else if (arg == "--active" && i + 1 < argc) {
            active = {false, false, false, false, false, false};
            for (int leg : parseLegs(argv[++i])) {
                if (leg >= 0 && leg < 6) active[leg] = true;
            }
        } else if (arg == "--quad-active" && i + 1 < argc) {
            std::vector<int> active_legs = parseLegs(argv[++i]);
            config.leg_radius = 220.0;
            config.leg_sitting_z = -40.0;
            config.corner_leg_angle_deg = 60.0;
            config.elongation = 1.0;
            config.neutral_feet = apk_model::makeNeutralFeet(220.0, -40.0, 60.0, 1.0, active_legs);
            active = {false, false, false, false, false, false};
            for (int leg : active_legs) {
                if (leg >= 0 && leg < 6) active[leg] = true;
            }
        } else if (arg == "--frame" && i + 1 < argc) {
            ProbeFrame frame;
            if (!parseFrame(argv[++i], frame)) {
                std::cerr << "invalid --frame value\n";
                return 2;
            }
            frames.push_back(frame);
        }
    }
    if (pulse_seed) {
        seedFromPulses(config, state, seed_pulses, body_z);
    } else {
        apk_model::initializeWalkState(config, state);
        state.body.body.xyz = {0.0, 0.0, body_z};
    }
    std::array<int, 18> last_pulses = seed_pulses;

    if (frames.empty()) {
        const std::array<double, 6> forward = {
            0.025,
            0.04875,
            0.0713125,
            0.092746875,
            0.11310953125,
            0.11310953125,
        };
        const std::array<double, 6> turn = {
            0.0125,
            0.024375,
            0.03565625,
            0.0463734375,
            0.056554765625,
            0.056554765625,
        };
        const std::array<double, 6> dt = {0.0, 297.0, 511.0, 512.0, 513.0, 514.0};
        const std::array<bool, 6> allow = {true, true, true, true, true, false};
        for (size_t i = 0; i < dt.size(); ++i) {
            ProbeFrame frame;
            frame.gait = 5;
            frame.style = 3;
            frame.dt = dt[i];
            frame.allow = allow[i];
            frame.command.forward = forward[i];
            frame.command.turn = turn[i];
            frames.push_back(frame);
        }
    }
    for (size_t i = 0; i < frames.size(); ++i) {
        const ProbeFrame& frame = frames[i];
        auto step = apk_model::walkStep(config, state, frame.command, frame.gait, frame.style,
                                        frame.dt, frame.allow, active);
        std::array<int, 18> pulses = toPulses(step);
        for (int leg = 0; leg < 6; ++leg) {
            if (active[leg]) continue;
            for (int pin : apk_model::ServoPinByApkLeg[leg]) {
                pulses[pin] = last_pulses[pin];
            }
        }
        last_pulses = pulses;
        std::cout << i
                  << " phase=" << step.phase
                  << " allow=" << (frame.allow ? "true" : "false")
                  << " gait=" << frame.gait
                  << " style=" << frame.style
                  << " dt=" << frame.dt
                  << " bodyY=" << state.body.body.xyz.y
                  << " bodyU=" << state.body.body.uvw.x
                  << " layerX=" << state.animation_layer.xyz.x
                  << " layerU=" << state.animation_layer.uvw.x
                  << " anchors=";
        for (bool anchor : state.anchor_active) std::cout << (anchor ? "1" : "0");
        std::cout << " pulses=";
        printPulses(pulses);
        std::cout << "\n";
    }
    return 0;
}
