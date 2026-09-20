/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Richard Wang (1698V)                                      */
/*    Modified:     5327V                                                     */
/*    Created:      July 9, 2023                                              */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "autonomous.h"
#include "config.h"

using namespace vex;

// Replacement for Voltage/src/main.cpp when using ONLY the no-IMU toggle
// routines. Keep a copy of your original main.cpp outside src/ for IMU routes.
// Copy BOTH override_toggle_red_no_imu.cpp and override_toggle_blue_no_imu.cpp
// into src/autons/. The original autonomous files can remain there too.
enum class ToggleAlliance { Red, Blue };
constexpr ToggleAlliance TOGGLE_ALLIANCE = ToggleAlliance::Red; // Choose Red or Blue.
void overrideToggleOnlyRedNoImu15();
void overrideToggleOnlyBlueNoImu15();

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  This section runs before autonomous and driver control begin.             */
/*  Initializes devices and drive encoders without an IMU or odometry task.  */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
    vexcodeInit();
    claw.initialize();

    // Battery Warning
    if (Brain.Battery.capacity() < 50) {
        Controller.rumble("...");
    }

    // No IMU calibration, IMU reads, or IMU-dependent odometry threads.
    left_chassis.stop(brake);
    right_chassis.stop(brake);
    resetChassis();
    Brain.Screen.print("Toggle auton: NO IMU");
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*                                                                           */
/*  This section controls the robot during the autonomous period.             */
/*  Runs the encoder-only toggle routine with its own motor fault checks.     */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
    left_chassis.stop(brake);
    right_chassis.stop(brake);
    intake.stop();
    lift.stop();
    // Reset the prior run's latch only at the start of a new autonomous run.
    // No legacy IMU watchdog is launched. Each routine still honors a new stop.
    SAFETY_STOP = false;
    if (!Competition.isEnabled() || !Competition.isAutonomous()) return;

    auto_start = Brain.Timer.system();

    if (TOGGLE_ALLIANCE == ToggleAlliance::Red) overrideToggleOnlyRedNoImu15();
    else overrideToggleOnlyBlueNoImu15();

    auto_end = Brain.Timer.system();
    printf("Autonomous Time: %d ms\n", auto_end - auto_start);
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*                                                                           */
/*  This section controls the robot during the driver control period.         */
/*  It handles the intake and arcade drivetrain control.                      */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
    stopChassis(coast);
    SAFETY_STOP = false;
    heading_correction = false;

    printf("DRIVER\n");

    bool clawButtonWasPressed = false;

    while (true) {
        // ===== INTAKE =====
        if (Controller.ButtonL1.pressing()) {
            intake.spin(IN, INTAKE_SPEED_RPM);
        } else if (Controller.ButtonL2.pressing()) {
            intake.spin(OUT, INTAKE_SPEED_RPM);
        } else {
            intake.stop();
        }

        // ===== LIFT =====
        bool liftUpPressed = Controller.ButtonR1.pressing();
        bool liftDownPressed = Controller.ButtonR2.pressing();

        if (liftUpPressed == liftDownPressed) {
            lift.stop();
        } else if (liftUpPressed) {
            lift.up();
        } else {
            lift.down();
        }

        // ===== PNEUMATIC CLAW =====
        bool clawButtonPressed = Controller.ButtonA.pressing();
        if (clawButtonPressed && !clawButtonWasPressed) {
            claw.toggle();
        }
        clawButtonWasPressed = clawButtonPressed;

        // ===== DRIVETRAIN =====
        double rawThrottle = deadband(Controller.Axis3.value(), 5);
        double rawTurn = deadband(Controller.Axis1.value(), 5);
        double ratio = std::abs(rawThrottle) >= std::abs(rawTurn) ? 0.9 : 0.6;
        double turn = rawTurn * ratio;

        driveChassis(to_volt(rawThrottle + turn), to_volt(rawThrottle - turn));

        wait(10, msec);
    }
}

/*---------------------------------------------------------------------------*/
/*                                   Main                                    */
/*                                                                           */
/*  Main sets up the competition callbacks for autonomous and driver control. */
/*  After setup, it stays in an infinite loop so the program keeps running.   */
/*---------------------------------------------------------------------------*/

int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();

    while (true) {
        wait(100, msec);
    }
}
