// VEXcode V5 / Voltage: BLUE alliance toggle-only autonomous (NO IMU).
// Blue: two forward press/retreat cycles; no turns or mechanism commands.
// Entry point: overrideToggleOnlyBlueNoImu15().
// Keep override_auton.cpp for the separate toggle-and-pin scoring routine.
//
// START: manually align the FRONT passive mechanism square to your toggle.
// Start close enough to reach it during PRESS_MS, without touching the toggle.
// Check the legal starting footprint/perimeter contact on the actual field.
// This routine does not drive from a remote starting position or turn around.
// NO IMU needed: compare left/right drive encoder travel for straight driving.
// Use integration/no_imu_main.cpp to skip the legacy IMU calibration and watchdog.
// Copying this file alone will NOT stop an old watchdog from braking the drive.
//
// The requested color mapping assumes the initial yellow orientation and a
// mechanism which advances exactly one face per forward stroke: red once,
// blue twice. No color/contact sensor verifies that a face actually changed.
// Tune PRESS_MS, PRESS_VOLTS and RETREAT_IN so every press seats ONE face and
// every retreat fully disengages/resets the mechanism. Counts are attempts.
// Encoder distance cannot correct wheel slip; test both colors on the field.

#include "config.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace override_toggle_blue_no_imu {
constexpr uint32_t DEADLINE_MS = 14800;
constexpr uint32_t PRESS_MS = 1800; // Includes travel back to toggle on cycle 2.
constexpr double PRESS_VOLTS = 2.4; // Gentle, bounded wall contact; tune physically.
constexpr double MAX_PRESS_TRAVEL_IN = 16.0; // Abort runaway travel, not a contact detector.
constexpr double RETREAT_IN = 8.0; // Must fully disengage the passive mechanism.
constexpr double RETREAT_VOLTS = 5.0;
constexpr uint32_t RETREAT_TIMEOUT_MS = 2500;
constexpr uint32_t RESET_PAUSE_MS = 250;

double clamp(double x, double low, double high) {
    return std::max(low, std::min(high, x));
}
uint32_t now() { return vex::timer::system(); }

class Routine {
    uint32_t started, sampled;
    double leftOrigin, rightOrigin, previousLeft, previousRight;
    double previousDistance, leftVolts, rightVolts;
    bool failed;
public:
    double distance, speed, sideError, dt;
    Routine() : started(now()), sampled(now()),
        leftOrigin(leftDistance()), rightOrigin(rightDistance()),
        previousLeft(leftOrigin), previousRight(rightOrigin),
        previousDistance((leftOrigin + rightOrigin) / 2),
        leftVolts(0), rightVolts(0), failed(false), distance(previousDistance),
        speed(0), sideError(0), dt(10) {}
    ~Routine() { stop(); }

    double leftDistance() {
        return left_chassis1.position(degrees) * wheel_distance_in / 360.0;
    }
    double rightDistance() {
        return right_chassis1.position(degrees) * wheel_distance_in / 360.0;
    }
    void stop() {
        left_chassis.stop(brake); right_chassis.stop(brake);
        leftVolts = rightVolts = 0;
    }
    bool fail(const char* reason) {
        if (!failed) printf("Toggle-only stopped: %s\n", reason);
        failed = true; stop(); return false;
    }
    bool sample() {
        if (failed) return false;
        if (now() - started >= DEADLINE_MS) return fail("14.8-second cutoff");
        if (!Competition.isEnabled() || !Competition.isAutonomous()) return fail("autonomous ended");
        if (SAFETY_STOP) return fail("stop requested; check for a legacy IMU watchdog");
        if (
            !left_chassis1.installed() || !left_chassis2.installed() || !left_chassis3.installed() ||
            !right_chassis1.installed() || !right_chassis2.installed() || !right_chassis3.installed())
            return fail("drivetrain motor fault");
        if (!std::isfinite(wheel_distance_in) || wheel_distance_in <= 0)
            return fail("invalid wheel distance conversion");
        const double left = leftDistance(), right = rightDistance();
        distance = (left + right) / 2.0;
        sideError = (left - leftOrigin) - (right - rightOrigin);
        if (!std::isfinite(distance) || !std::isfinite(sideError)) return fail("invalid encoder reading");
        if (std::fabs(sideError) > 3.0) return fail("drive sides diverged; check obstruction or slip");
        const uint32_t elapsed = now() - sampled;
        if (elapsed > 100) return fail("control loop delayed");
        if (elapsed > 0) {
            dt = elapsed;
            const double rawSpeed = (distance - previousDistance) * 1000.0 / dt;
            if (std::fabs(left - previousLeft) * 1000 / dt > 150 ||
                std::fabs(right - previousRight) * 1000 / dt > 150) return fail("encoder jump");
            speed += dt / (30.0 + dt) * (rawSpeed - speed);
            previousDistance = distance; previousLeft = left; previousRight = right; sampled = now();
        }
        return true;
    }
    void command(double forward, double cap) {
        // Signed travel correction works in both directions. It balances wheel
        // rotation, not physical heading: equal wheel slip remains unobservable.
        const double correction = clamp(-sideError * 0.6, -0.6, 0.6);
        double left = forward + correction, right = forward - correction;
        const double peak = std::max(std::fabs(left), std::fabs(right));
        if (peak > cap) { left *= cap / peak; right *= cap / peak; }
        auto ramp = [this](double target, double previous) {
            if (target * previous < 0) target = 0;
            const double step = 0.4 * dt / 10.0;
            return previous + clamp(target - previous, -step, step);
        };
        leftVolts = clamp(ramp(left, leftVolts), -cap, cap);
        rightVolts = clamp(ramp(right, rightVolts), -cap, cap);
        left_chassis.spin(fwd, leftVolts, volt);
        right_chassis.spin(fwd, rightVolts, volt);
    }
    bool pause(uint32_t duration) {
        const uint32_t start = now();
        while (now() - start < duration) {
            if (!sample()) return false;
            wait(10, msec);
        }
        return sample();
    }
    bool press() {
        if (!sample()) return false;
        leftOrigin = leftDistance(); rightOrigin = rightDistance(); sideError = 0;
        const double origin = distance;
        const uint32_t start = now();
        while (now() - start < PRESS_MS) {
            if (!sample()) return false;
            if (std::fabs(distance - origin) > MAX_PRESS_TRAVEL_IN)
                return fail("forward travel limit; check starting gap or wheel slip");
            // Wall contact is intentional: no distance integral or stall retry.
            command(PRESS_VOLTS, PRESS_VOLTS + 0.6);
            wait(10, msec);
        }
        stop();
        return sample();
    }
    bool retreat() {
        if (!sample()) return false;
        leftOrigin = leftDistance(); rightOrigin = rightDistance(); sideError = 0;
        const double target = distance - RETREAT_IN;
        double settled = 0, progress = distance;
        uint32_t progressTime = now();
        const uint32_t start = now();
        while (now() - start < RETREAT_TIMEOUT_MS) {
            if (!sample()) return false;
            const double error = target - distance;
            // Encoder PD with speed damping; no integral windup.
            const double output = clamp(1.4 * error - 0.04 * speed, -RETREAT_VOLTS, RETREAT_VOLTS);
            settled = std::fabs(error) < 0.35 && std::fabs(speed) < 2.0 &&
                      std::fabs(sideError) < 0.4 ? settled + dt : 0;
            if (settled >= 80) { stop(); return pause(RESET_PAUSE_MS); }
            if (std::fabs(distance - progress) > 0.15 || std::fabs(output) < 2.0) {
                progress = distance; progressTime = now();
            } else if (now() - progressTime > 600) return fail("retreat stalled");
            command(output, RETREAT_VOLTS);
            wait(10, msec);
        }
        return fail("retreat timeout");
    }
};

void run(unsigned cycles) {
    Routine robot;
    for (unsigned i = 0; i < cycles; ++i) {
        if (!robot.press() || !robot.retreat()) return;
        printf("Toggle-only: completed press/retreat %u of %u\n", i + 1, cycles);
    }
    // Retreat after the LAST stroke too: a robot touching the toggle prevents
    // it being counted as set. No further movement until driver control.
    robot.stop();
}
} // namespace override_toggle_blue_no_imu

void overrideToggleOnlyBlueNoImu15() { override_toggle_blue_no_imu::run(2); }
