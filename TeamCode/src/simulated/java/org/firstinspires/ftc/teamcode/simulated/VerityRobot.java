package org.firstinspires.ftc.teamcode.simulated;

import org.ngicollective.testframework.behavior.ImuBehaviors;
import org.ngicollective.testframework.hardware.FakeHardwareMap;
import org.ngicollective.testframework.hardware.SimulatedRobot;

/**
 * The simulated stand-in for the team's robot: the same device names the real robot configuration
 * file declares, so unmodified OpModes find their hardware.
 *
 * <p>Single source of truth for the headless tests, the dashboard, and the simulated build of the
 * app itself. Rename a motor here and every OpMode that looks it up fails in the same place.</p>
 *
 * <p>Speeds are the goBILDA 5202 312 RPM yellow jacket's published free speed and encoder
 * resolution. They are a starting point for relative comparisons, not a calibrated model of this
 * robot's drivetrain under load.</p>
 */
public class VerityRobot implements SimulatedRobot {

    private static final double DRIVE_MOTOR_RPM = 312.0;
    private static final double DRIVE_TICKS_PER_REVOLUTION = 537.7;

    @Override
    public String name() {
        return "Verity";
    }

    @Override
    public FakeHardwareMap create() {
        FakeHardwareMap hardware = FakeHardwareMap.builder()
                // followingYawRate rather than stationary: the dashboard and tests can then turn
                // the robot by writing a turn rate, instead of teleporting the heading.
                .addImu("imu", ImuBehaviors.followingYawRate())
                .addMotor("FL")
                .addMotor("FR")
                .addMotor("BL")
                .addMotor("BR")
                .build();

        for (String wheel : new String[]{"FL", "FR", "BL", "BR"}) {
            hardware.motor(wheel).state().setMaxSpeed(DRIVE_MOTOR_RPM, DRIVE_TICKS_PER_REVOLUTION);
        }
        return hardware;
    }
}
