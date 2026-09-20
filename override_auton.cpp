// Voltage / VEXcode V5 C++ -- standalone 15-second end-wall autonomous.
// Save as src/autons/override_auton.cpp; add `void overrideAutonomous15();`
// to include/autonomous.h, then replace right_5() in autonomous() with it.
// Uses the CURRENT repository's config.h, motors, inertial_sensor and claw.
// Does not require the unfinished PID/control upgrade from the earlier task.
//
// START (Appendix A, audience-view coordinates, inches):
// Red: robot center X=70.20, rear bumper touching bottom wall, front into field.
// Blue: rotate that entire setup 180 degrees, at the opposite end wall.
// The passive FRONT toggler faces into the field initially; REAR claw is low.
// Start legally clear of the toggle and scoring objects (SG1). Seat the lift
// fully down by hand BEFORE enabling: this routine zeros its encoders there.
// Put the required preload on the floor touching the robot's OUTER/right side
// so it is left behind when driving out. Verify it is completely released
// before collecting the wall pin (SG5/SG6: possession is limited to one pin).
//
// ROUTE: pull out -> face wall -> push front toggler fully -> clear toggle ->
// angled rear-claw approach to LEFT wall cup-and-pin stack -> close/lift ->
// place that stack ON the existing yellow pin in the RIGHT black neutral goal
// -> release -> pull clear and stop. The bottom cup must nest over the existing
// pin; setting a cup onto an empty goal would not establish a scoring stack.
// Never remove the pin already in that neutral goal (SG10). Clearing the toggle is required
// for ownership (SC4/SC5). The code cannot sense toggle color or pin capture.
//
// Sources: supplied Override v2.0, PDF pages 20, 25-26, 29-34, 102, 107;
// supplied 5327V notebook, PDF pages 47, 64, 71-72, 78-80, 149-150.
// Official companion: https://www.vexrobotics.com/override-manual
//
// REQUIRED TUNING: fill the two lift setpoints below (motor DEGREES), and
// measure rear claw reach / chassis dimensions / toggle contact distance.
// Then set ROBOT_GEOMETRY_VERIFIED=true after checking the legal starting
// footprint, deployed reach and clearances. The route's timing also needs a
// field test; slow lifts or drives may trigger the deadline before scoring.
// NAN deliberately prevents running with invented lift measurements.
// These are starting control gains, not field-tested performance guarantees.
// Encoder odometry cannot measure wheel slip without the absent trackers;
// recheck pickup/goal alignment on the real field before using in a match.

#include "config.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace override_auto {
constexpr double PI = 3.14159265358979323846;
constexpr uint32_t DEADLINE_MS = 14800;

// Lift encoder positions relative to the physically lowered starting position.
constexpr double PICKUP_LIFT_DEG = 0.0;  // Confirmed: claw picks up at its resting position.
constexpr double CARRY_LIFT_DEG = NAN;   // Bottom of held cup clears TOP of goal's existing pin.
constexpr double PLACE_LIFT_DEG = NAN;   // Lower cup nests over that existing yellow pin.
constexpr double LIFT_SPEED_RPM = 150.0;
constexpr double LIFT_TOL_DEG = 8.0;
constexpr double DRIVE_KP = 1.4, DRIVE_KI = 0.0, DRIVE_KD = 4.0;
constexpr double TURN_KP = 0.25, TURN_KI = 0.0, TURN_KD = 1.58;

// NOMINAL ROBOT GEOMETRY -- measure; these are not notebook measurements.
// Rear reach is axle/turn-center -> vertical axis of the held cup-and-pin stack.
// The nominal 15-inch reach assumes a deployable claw that fits inside the
// 18-inch starting footprint; a fixed rear extension requires revised geometry.
// This particular angled pickup requires adequate reach; short rear claws
// need a different approach. validSetup() checks wall and goal clearance.
constexpr double HALF_WIDTH_IN = 9.0;
constexpr bool ROBOT_GEOMETRY_VERIFIED = false;
constexpr double FRONT_BODY_IN = 9.0;
constexpr double REAR_BODY_IN = 9.0;
constexpr double REAR_CLAW_REACH_IN = 15.0;
constexpr double TOGGLE_CONTACT_IN = 10.0; // Center -> wall, fully engaged.
constexpr double PICKUP_HEADING_DEG = 40.0;
constexpr double PICKUP_RUN_IN = 12.0;

// Field coordinates from Appendix A12, not nominal 24" tile spacing.
// Local +Y = into field from this end wall; +X = robot's right at start.
constexpr double TOGGLE_X = 70.20;
constexpr double WALL_PIN_X = 46.66;
constexpr double WALL_PIN_Y = 1.58;
constexpr double PICKUP_OBSTACLE_GOAL_X = 46.66; // Empty alliance goal beside pickup.
constexpr double GOAL_X = 93.75; // Black goal starts with a yellow/yellow pin.
constexpr double GOAL_Y = 23.11;
constexpr double CLEAR_Y = 24.0;
constexpr double GOAL_RADIUS = 3.22; // Circumscribed radius from Appendix A7.

double limit(double n, double lo, double hi) {
    return std::max(lo, std::min(hi, n));
}
double radians(double d) { return d * PI / 180.0; }
double wrap(double d) {
    d = std::fmod(d, 360.0);
    if (d > 180) d -= 360;
    if (d < -180) d += 360;
    return d;
}
double clockMs() { return vex::timer::system(); }

// Small time-aware controller local to this routine. Gains use the existing
// template's 10 ms convention. Filtered derivative; bounded integral and output.
struct ControllerPID {
    double kp, ki, kd, maxV, sum, previous, rate;
    bool first;
    ControllerPID(double p, double i, double d, double cap)
        : kp(p), ki(i), kd(d), maxV(cap), sum(0), previous(0), rate(0), first(true) {}
    double update(double error, double dt) {
        if (first) { previous = error; first = false; }
        const double rawRate = (error - previous) * 1000.0 / dt;
        rate += dt / (20.0 + dt) * (rawRate - rate);
        previous = error;
        if (std::fabs(error) > 3 || error * sum < 0 || std::fabs(error) < 0.25)
            sum = 0;
        const double pd = kp * error + kd * rate * 0.01;
        if (ki != 0 && std::fabs(error) <= 3 && std::fabs(error) >= 0.25) {
            const double candidate = limit(sum + error * dt / 10.0, -1.5 / ki, 1.5 / ki);
            const double output = pd + ki * candidate;
            if ((output <= maxV || error < 0) && (output >= -maxV || error > 0))
                sum = candidate;
        }
        return limit(pd + ki * sum, -maxV, maxV);
    }
};

class Routine {
    uint32_t started;
    double imuZero, prevLeft, prevRight, prevHeading, lastSample;
    double leftV, rightV;
    bool failed;
public:
    double x, y, heading, traveled, dt;
    Routine()
        : started(vex::timer::system()), imuZero(inertial_sensor.rotation(degrees)),
          prevLeft(left_chassis1.position(degrees)), prevRight(right_chassis1.position(degrees)),
          prevHeading(0), lastSample(clockMs()), leftV(0), rightV(0), failed(false),
          x(TOGGLE_X), y(REAR_BODY_IN), heading(0), traveled(0), dt(10) {}
    ~Routine() { stop(); lift.stop(); intake.stop(); }

    void stop() {
        left_chassis.stop(brake); right_chassis.stop(brake);
        leftV = rightV = 0;
    }
    bool fail(const char* reason) {
        if (!failed) printf("Override auton stopped: %s at %lu ms\n", reason,
                            static_cast<unsigned long>(vex::timer::system() - started));
        failed = true; stop(); lift.stop(); intake.stop();
        // Keep pneumatic grip unchanged on abort; do not drop a carried pin.
        return false;
    }
    bool alive() {
        if (failed) return false;
        if (vex::timer::system() - started >= DEADLINE_MS) return fail("14.8 s deadline");
        if (!Competition.isEnabled() || !Competition.isAutonomous()) return fail("autonomous ended");
        if (SAFETY_STOP || !inertial_sensor.installed() || inertial_sensor.isCalibrating() ||
            !left_chassis1.installed() || !left_chassis2.installed() || !left_chassis3.installed() ||
            !right_chassis1.installed() || !right_chassis2.installed() || !right_chassis3.installed() ||
            !LeftLiftMotor.installed() || !RightLiftMotor.installed()) return fail("device fault");
        return true;
    }
    bool sample() {
        if (!alive()) return false;
        const double time = clockMs();
        const double elapsed = time - lastSample;
        if (elapsed == 0) { dt = 10; return true; }
        if (elapsed < 0 || elapsed > 100) return fail("control loop delayed");
        dt = elapsed; lastSample = time;
        const double l = left_chassis1.position(degrees), r = right_chassis1.position(degrees);
        heading = inertial_sensor.rotation(degrees) - imuZero;
        if (!std::isfinite(l) || !std::isfinite(r) || !std::isfinite(heading)) return fail("invalid sensor reading");
        const double dl = (l - prevLeft) * wheel_distance_in / 360.0;
        const double dr = (r - prevRight) * wheel_distance_in / 360.0;
        const double dh = radians(wrap(heading - prevHeading));
        if (std::fabs(dl) / dt > 0.2 || std::fabs(dr) / dt > 0.2 || std::fabs(dh) / dt > 0.03)
            return fail("sensor jump");
        const double distance = (dl + dr) / 2.0;
        const double chord = std::fabs(dh) < 1e-6 ? 1.0 : 2.0 * std::sin(dh / 2.0) / dh;
        const double mid = radians(prevHeading) + dh / 2.0;
        x += distance * chord * std::sin(mid);
        y += distance * chord * std::cos(mid);
        traveled += distance;
        prevLeft = l; prevRight = r; prevHeading = heading;
        return true;
    }
    bool command(double left, double right, double cap) {
        if (!alive()) return false;
        if (!std::isfinite(left) || !std::isfinite(right)) return fail("invalid motor command");
        cap = limit(cap, 0, 12);
        const double peak = std::max(std::fabs(left), std::fabs(right));
        if (peak > cap) { left *= cap / peak; right *= cap / peak; }
        // 0.4 V / 10 ms acceleration; 0.7 V / 10 ms braking. Reverse via zero.
        auto slew = [this](double wanted, double previous) {
            if (wanted * previous < 0) wanted = 0;
            const double step = (std::fabs(wanted) > std::fabs(previous) ? 0.4 : 0.7) * dt / 10;
            return previous + limit(wanted - previous, -step, step);
        };
        leftV = limit(slew(left, leftV), -cap, cap);
        rightV = limit(slew(right, rightV), -cap, cap);
        left_chassis.spin(fwd, leftV, volt); right_chassis.spin(fwd, rightV, volt);
        return true;
    }
    bool pause(unsigned ms) {
        const double start = clockMs();
        while (clockMs() - start < ms) {
            if (!sample()) return false;
            wait(10, msec);
        }
        return sample();
    }
    bool move(double inches, double targetHeading, unsigned timeout, double cap = 7) {
        if (!sample()) return false;
        const double startDistance = traveled, start = clockMs();
        double settled = 0, progress = traveled, progressTime = start;
        ControllerPID distance(DRIVE_KP, DRIVE_KI, DRIVE_KD, cap);
        ControllerPID yaw(heading_correction_kp, 0, heading_correction_kd, 2.5);
        while (clockMs() - start < timeout) {
            if (!sample()) return false;
            const double error = inches - (traveled - startDistance);
            const double yawError = wrap(targetHeading - heading);
            const double forward = distance.update(error, dt), turn = yaw.update(yawError, dt);
            settled = std::fabs(error) < 0.45 && std::fabs(distance.rate) < 2.0 &&
                      std::fabs(yawError) < 1.5 && std::fabs(yaw.rate) < 10 ? settled + dt : 0;
            if (settled >= 60) { stop(); return true; }
            if (std::fabs(traveled - progress) > 0.15 || std::fabs(forward) < 3) {
                progress = traveled; progressTime = clockMs();
            } else if (clockMs() - progressTime > 650) return fail("drive stalled");
            if (!command(forward + turn, forward - turn, cap)) return false;
            wait(10, msec);
        }
        return fail("drive did not reach target");
    }
    bool turn(double target, unsigned timeout = 1800) {
        if (!sample()) return false;
        target = heading + wrap(target - heading);
        const double start = clockMs(); double settled = 0;
        ControllerPID pid(TURN_KP, TURN_KI, TURN_KD, 9);
        while (clockMs() - start < timeout) {
            if (!sample()) return false;
            const double error = target - heading, output = pid.update(error, dt);
            settled = std::fabs(error) < 1.2 && std::fabs(pid.rate) < 8 ? settled + dt : 0;
            if (settled >= 60) { stop(); return true; }
            if (!command(output, -output, 9)) return false;
            wait(10, msec);
        }
        return fail("turn did not settle");
    }
    bool go(double tx, double ty, unsigned timeout = 1600, bool backwards = false) {
        if (!sample()) return false;
        const double dx = tx - x, dy = ty - y;
        const double h = std::atan2(dx, dy) * 180 / PI + (backwards ? 180 : 0);
        if (std::hypot(dx, dy) < 0.45) return true;
        return turn(h) && move((backwards ? -1 : 1) * std::hypot(dx, dy), h, timeout);
    }
    bool pushToggle() {
        const double start = clockMs();
        // Deliberate low-voltage wall contact; bounded so encoders cannot wind up.
        while (clockMs() - start < 350) {
            if (!sample()) return false;
            const double correction = limit(0.3 * wrap(180 - heading), -0.6, 0.6);
            if (!command(2.4 + correction, 2.4 - correction, 3.0)) return false;
            wait(10, msec);
        }
        stop();
        // No contact switch/color sensor: don't falsely relocalize from assumed contact.
        return pause(60);
    }
    bool raiseTo(double target, unsigned timeout = 1400) {
        stop();
        if (!alive()) return false;
        // Each V5 motor runs its built-in position controller concurrently.
        LeftLiftMotor.spinToPosition(target, degrees, LIFT_SPEED_RPM, rpm, false);
        RightLiftMotor.spinToPosition(target, degrees, LIFT_SPEED_RPM, rpm, false);
        const double start = clockMs(); double settled = 0, mismatchTime = 0;
        double previousL = LeftLiftMotor.position(degrees), previousR = RightLiftMotor.position(degrees);
        while (clockMs() - start < timeout) {
            if (!sample()) return false;
            const double l = LeftLiftMotor.position(degrees), r = RightLiftMotor.position(degrees);
            if (!std::isfinite(l) || !std::isfinite(r)) return fail("invalid lift reading");
            mismatchTime = std::fabs(l - r) > 60 ? mismatchTime + dt : 0;
            if (mismatchTime > 150) return fail("cascade sides out of sync");
            const bool slow = std::fabs(l - previousL) / dt < 0.08 && std::fabs(r - previousR) / dt < 0.08;
            settled = std::fabs(target - l) < LIFT_TOL_DEG && std::fabs(target - r) < LIFT_TOL_DEG && slow ? settled + dt : 0;
            if (settled >= 80) { lift.stop(); return true; }
            previousL = l; previousR = r;
            wait(10, msec);
        }
        return fail("lift did not reach target");
    }
};

bool validSetup() {
    if (!ROBOT_GEOMETRY_VERIFIED) return false;
    if (!std::isfinite(PICKUP_LIFT_DEG) || !std::isfinite(CARRY_LIFT_DEG) ||
        !std::isfinite(PLACE_LIFT_DEG) || PICKUP_LIFT_DEG < 0 || PLACE_LIFT_DEG < 0 ||
        CARRY_LIFT_DEG <= std::max(PICKUP_LIFT_DEG, PLACE_LIFT_DEG)) return false;
    if (!std::isfinite(wheel_distance_in) || wheel_distance_in <= 0) return false;
    if (2 * HALF_WIDTH_IN > 18 || FRONT_BODY_IN + REAR_BODY_IN > 18 ||
        FRONT_BODY_IN + REAR_CLAW_REACH_IN > 24) return false;
    const double h = radians(PICKUP_HEADING_DEG);
    const double px = WALL_PIN_X + REAR_CLAW_REACH_IN * std::sin(h);
    const double py = WALL_PIN_Y + REAR_CLAW_REACH_IN * std::cos(h);
    const double wallClearance = py - REAR_BODY_IN * std::cos(h) - HALF_WIDTH_IN * std::sin(h);
    const double goalSideClearance = std::fabs((PICKUP_OBSTACLE_GOAL_X - px) * std::cos(h) -
                                               (GOAL_Y - py) * std::sin(h)) - HALF_WIDTH_IN - GOAL_RADIUS;
    const double ax = px + PICKUP_RUN_IN * std::sin(h);
    const double ay = py + PICKUP_RUN_IN * std::cos(h);
    const double bodyRadius = std::hypot(HALF_WIDTH_IN, std::max(FRONT_BODY_IN, REAR_BODY_IN));
    const double sweepRadius = std::max(bodyRadius, REAR_CLAW_REACH_IN + 1.58);
    // The carried stack is above the empty alliance goal; check the chassis
    // sweep here. Verify the open claw's height clears that goal before pickup.
    const bool turnClear = ay > sweepRadius + 0.5 &&
        std::hypot(ax - PICKUP_OBSTACLE_GOAL_X, ay - GOAL_Y) > bodyRadius + GOAL_RADIUS + 0.5;
    return wallClearance >= 0.25 && goalSideClearance >= 0.5 && turnClear;
}
} // namespace override_auto

void overrideAutonomous15() {
    using namespace override_auto;
    Routine run;
    if (!validSetup()) {
        run.fail("fill measured lift setpoints and check rear-claw geometry first");
        return;
    }
    if (!run.alive()) return;
    LeftLiftMotor.setPosition(0, degrees);
    RightLiftMotor.setPosition(0, degrees);
    intake.stop(); claw.open();

    // 1. Clear the wall, turn the passive FRONT toggler toward it, push fully.
    if (!run.move(CLEAR_Y - REAR_BODY_IN, 0, 1400)) return;
    if (!run.turn(180)) return;
    if (!run.move(CLEAR_Y - TOGGLE_CONTACT_IN, 180, 1400, 5)) return;
    if (!run.pushToggle()) return;
    // Leave the toggle untouched so its color is counted (SC4).
    if (!run.move(-(CLEAR_Y - run.y), 180, 1400, 6)) return;

    // 2. Pick the entire upright cup-and-pin stack at the confirmed resting
    // height. Approach at an angle to avoid the alliance goal in front of it.
    const double h = radians(PICKUP_HEADING_DEG);
    const double approachX = WALL_PIN_X + (REAR_CLAW_REACH_IN + PICKUP_RUN_IN) * std::sin(h);
    const double approachY = WALL_PIN_Y + (REAR_CLAW_REACH_IN + PICKUP_RUN_IN) * std::cos(h);
    if (!run.go(approachX, approachY, 1200, true)) return;
    if (!run.turn(PICKUP_HEADING_DEG)) return;
    if (!run.raiseTo(PICKUP_LIFT_DEG)) return;
    if (!run.move(-PICKUP_RUN_IN, PICKUP_HEADING_DEG, 1500, 4)) return;
    claw.close();
    if (!run.pause(200)) return; // Notebook measured ~0.12 s; allow margin.
    if (!run.raiseTo(CARRY_LIFT_DEG)) return; // Entire stack clear BEFORE driving.
    if (!run.move(PICKUP_RUN_IN, PICKUP_HEADING_DEG, 1400, 5)) return;

    // 3. Stack onto the existing pin in our quadrant's black neutral goal.
    // Rear faces +X. Keep the whole robot on our side of the autonomous line.
    if (!run.go(GOAL_X - REAR_CLAW_REACH_IN - 5.0, GOAL_Y, 1400, true)) return;
    if (!run.turn(-90)) return;
    if (!run.move(-5.0, -90, 900, 3)) return;
    if (!run.sample()) return;
    const double heldX = run.x - REAR_CLAW_REACH_IN * std::sin(radians(run.heading));
    const double heldY = run.y - REAR_CLAW_REACH_IN * std::cos(radians(run.heading));
    if (std::hypot(heldX - GOAL_X, heldY - GOAL_Y) > 0.8) {
        run.fail("estimated claw alignment outside goal tolerance");
        return;
    }
    if (!run.raiseTo(PLACE_LIFT_DEG)) return;
    claw.open();
    if (!run.pause(200)) return;
    if (!run.move(4.0, -90, 800, 3)) return; // Clear the placed stack before stopping.
    run.stop(); lift.stop();
    printf("Override auton sequence complete; verify pin/toggle visually.\n");
}
