package org.firstinspires.ftc.teamcode.simulated;

import org.ngicollective.testframework.behavior.ImuBehaviors;
import org.ngicollective.testframework.hardware.FakeHardwareMap;
import org.ngicollective.testframework.hardware.SimulatedRobot;
import org.ngicollective.testframework.sim.FieldConfig;
import org.ngicollective.testframework.sim.MotorConfig;
import org.ngicollective.testframework.sim.RobotConfig;
import org.ngicollective.testframework.sim.SimConfigFiles;

import java.util.Map;

/**
 * The simulated stand-in for the team's robot: the same device names the real robot configuration
 * file declares, so unmodified OpModes find their hardware.
 *
 * <p>Single source of truth for the headless tests, the dashboard, and the simulated build of the
 * app itself. Rename a motor here and every OpMode that looks it up fails in the same place.</p>
 *
 * <p>The numbers live in {@code TeamCode/robot-config/verity.json} rather than in this file. They
 * are physics: the drive model integrates them into a field pose, and the 3D scene draws the robot
 * they describe. Keeping them in one data file is what stops the encoder resolution from being
 * written down three times &mdash; once here, once in a layout file, once in the browser &mdash;
 * and drifting apart. The device <em>names</em> stay in code, because a rename must break a build.</p>
 *
 * <p>Motor directions are deliberately absent from the config: {@code DriveHardware} sets them at
 * runtime, and a second copy here would be free to disagree with the robot that actually ships.</p>
 */
public class VerityRobot implements SimulatedRobot {

    private static final String CONFIG_NAME = "verity";

    private final RobotConfig config;
    private final FieldConfig field;

    public VerityRobot() {
        this(SimConfigFiles.robot(CONFIG_NAME), SimConfigFiles.field());
    }

    /** For tests that want a robot or a field that differs from the one on disk. */
    public VerityRobot(RobotConfig config, FieldConfig field) {
        this.config = config;
        this.field = field;
    }

    @Override
    public String name() {
        return config.name();
    }

    @Override
    public FakeHardwareMap create() {
        FakeHardwareMap.Builder builder = FakeHardwareMap.builder()
                // followingChassis rather than followingYawRate: heading is now derived from the
                // wheels, so the IMU reports what the chassis actually did. The rate-following and
                // stationary presets remain selectable, and selecting one is now fault injection --
                // an IMU that disagrees with the drivetrain on purpose.
                .addImu(config.imuName(), ImuBehaviors.followingChassis());
        for (String motor : config.motors().keySet()) {
            builder.addMotor(motor);
        }

        FakeHardwareMap hardware = builder.withDrivetrain(config, field).build();
        for (Map.Entry<String, MotorConfig> entry : config.motors().entrySet()) {
            MotorConfig motor = entry.getValue();
            hardware.motor(entry.getKey())
                    .state()
                    .setMaxSpeed(motor.rpm(), motor.ticksPerRevolution());
        }
        return hardware;
    }

    /** The robot this configuration describes, for callers that need its geometry. */
    public RobotConfig config() {
        return config;
    }

    /** The field this robot is simulated on. */
    public FieldConfig field() {
        return field;
    }
}
