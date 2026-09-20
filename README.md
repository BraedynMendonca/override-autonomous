# Override autonomous routines

These are VEXcode V5 C++ routines for the Voltage project's existing `config.h` and device mapping, not a complete standalone robot project.

| File | Entry point | Routine |
| --- | --- | --- |
| `override_auton.cpp` | `overrideAutonomous15()` | Original end-wall toggle, rear-claw cup-and-pin pickup, and goal stacking. |
| `override_toggle_red_auton.cpp` | `overrideToggleOnlyRed15()` | Red alliance: one forward toggle press, then back away and stop. |
| `override_toggle_blue_auton.cpp` | `overrideToggleOnlyBlue15()` | Blue alliance: two forward toggle presses, backing away after each, then stop. |
| `override_toggle_red_no_imu.cpp` | `overrideToggleOnlyRedNoImu15()` | **No IMU:** red alliance, one press and retreat. |
| `override_toggle_blue_no_imu.cpp` | `overrideToggleOnlyBlueNoImu15()` | **No IMU:** blue alliance, two presses and retreats. |

## Run the new no-IMU versions

The original three autonomous files are preserved. The new files use the drive motors' built-in encoders; no inertial sensor or tracking wheels are required.

**Copying only a new routine will not fix `IMU DC`: the old `main.cpp` starts a separate watchdog that stops the drivetrain when the IMU is missing.** Use the matching startup file too:

1. Copy `override_toggle_red_no_imu.cpp` and `override_toggle_blue_no_imu.cpp` into your Voltage project's `src/autons/` folder. Keep the original routine files there if wanted.
2. Back up your project's existing `src/main.cpp` **outside `src/`** so it is not compiled twice.
3. Copy `integration/no_imu_main.cpp` into the project as **`src/main.cpp`**, replacing the startup file. This is based on Voltage's existing main file and preserves its driver controls. Its MIT attribution is in `integration/LICENSE`.
4. Near the top of that new `main.cpp`, select your alliance:

   ```cpp
   constexpr ToggleAlliance TOGGLE_ALLIANCE = ToggleAlliance::Red;
   // For blue, change Red to Blue. Red presses once; blue presses twice.
   ```

5. Rebuild and download the entire project to the Brain, then restart it. The startup screen should show `Toggle auton: NO IMU`.

This startup file skips IMU calibration, IMU-based odometry and the IMU watchdog. It only selects the new no-IMU routines. Restore your backed-up startup file to use the original IMU routines again. It does not change ports or driver button mappings.

The no-IMU routines compare signed left/right encoder travel to keep both sides moving together, and use encoder distance for backing away. They still stop for disconnected drive motors, invalid encoders, excessive side-to-side travel difference, a new safety-stop request, field disable, stalled retreat, timeout or the 14.8-second deadline. Without an IMU, they cannot measure actual heading or correct wheel slip; line up squarely and field-test the press time and retreat distance.

## Select an original IMU routine

1. Copy the desired original `.cpp` file(s) into your Voltage project's `src/autons/` folder. All routine files can coexist.
2. Add these declarations to `include/autonomous.h`:

   ```cpp
   void overrideAutonomous15();
   void overrideToggleOnlyRed15();
   void overrideToggleOnlyBlue15();
   ```

3. Inside `autonomous()` in `src/main.cpp`, replace `right_5();` with **one** selected call:

   ```cpp
   overrideToggleOnlyRed15(); // Red alliance: one press/retreat cycle.
   // OR overrideToggleOnlyBlue15(); // Blue alliance: two cycles.
   // OR overrideAutonomous15(); // Original toggle-and-pin scoring routine.
   ```

## Original IMU toggle-only setup

- Manually line up the front passive toggle mechanism square to the toggle. This routine drives straight forward and backward; it does not turn or find the toggle.
- Verify legal starting placement, including perimeter contact, starting size and no initial toggle contact. If your robot cannot legally face the toggle at the start, this straight-only route requires a different setup or an additional approach routine.
- The one-stroke red / two-stroke blue mapping follows the requested mechanism behavior and assumes the toggle begins yellow. Confirm each forward stroke advances exactly one face and the backward stroke resets your mechanism without reversing the toggle.
- Tune `PRESS_MS`, `PRESS_VOLTS`, and `RETREAT_IN` in each alliance file. Each file is self-contained and can be installed independently. The initial values are 1.8 seconds at 2.4 V and an 8-inch retreat. The forward travel limit is an abort guard, not a contact detector; wheel spin can trigger it.
- Leave the existing IMU calibration in `pre_auton()`. The routine holds the starting heading and uses drive encoders for the retreat. It does not require tracking wheels or lift calibration, and sends no lift, intake, or claw commands.
- Both variants stop on field disable, drivetrain/IMU faults, retreat stalls/timeouts, or the 14.8-second cutoff. They cannot verify toggle color or fully compensate for wheel slip. Field-test before a match.

The original scoring routine remains unchanged and still requires its lift heights and geometry to be configured separately.
