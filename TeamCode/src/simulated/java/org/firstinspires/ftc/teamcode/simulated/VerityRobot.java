package org.firstinspires.ftc.teamcode.simulated;

import org.ngicollective.testframework.behavior.ImuBehaviors;
import org.ngicollective.testframework.camera.CameraIntrinsics;
import org.ngicollective.testframework.camera.Pose3d;
import org.ngicollective.testframework.camera.SceneFrameSource;
import org.ngicollective.testframework.camera.SimulatedCamera;
import org.ngicollective.testframework.camera.SimulatedScene;
import org.ngicollective.testframework.camera.Vec3;
import org.ngicollective.testframework.hardware.FakeHardwareMap;
import org.ngicollective.testframework.hardware.SimulatedRobot;
import org.ngicollective.testframework.season.BioBuzzField;
import org.ngicollective.testframework.sim.CameraConfig;
import org.ngicollective.testframework.sim.FieldConfig;
import org.ngicollective.testframework.sim.MotorConfig;
import org.ngicollective.testframework.sim.Pose2d;
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

    /**
     * The camera's optics before an OpMode has chosen a resolution: a nominal 60&deg; webcam.
     *
     * <p>Not a placeholder. Once {@code VisionPortal} starts streaming, the simulated camera
     * swaps in the SDK's own calibration for the resolution the OpMode asked for, so that frames
     * are drawn through the very lens the pose solver inverts. This is what the plain-JVM tests,
     * which have no Android calibration database to consult, render through.</p>
     */
    private static final CameraIntrinsics NOMINAL_OPTICS =
            CameraIntrinsics.approximate(640, 480);

    private final RobotConfig config;
    private final FieldConfig field;
    private final SimulatedScene scene;

    public VerityRobot() {
        this(SimConfigFiles.robot(CONFIG_NAME), SimConfigFiles.field());
    }

    /** For tests that want a robot or a field that differs from the one on disk. */
    public VerityRobot(RobotConfig config, FieldConfig field) {
        this(config, field, BioBuzzField.official());
    }

    /**
     * The same, looking at a particular arrangement of the field.
     *
     * <p>The official field is the default because it needs no file to be right. A scenario is
     * for the arrangements that are a choice &mdash; a HIVE tipped the other way, balls left where
     * they broke autonomous last weekend &mdash; and
     * {@code SimConfigFiles.scenario(name).scene()} is where one comes from.</p>
     */
    public VerityRobot(RobotConfig config, FieldConfig field, SimulatedScene scene) {
        this.config = config;
        this.field = field;
        this.scene = scene;
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

        // The camera's view follows the robot, so its frame source needs the pose that the map it
        // is being added to will own. Hence the holder: the map cannot exist before the devices
        // that go in it, and the camera cannot read a pose before the map exists.
        final FakeHardwareMap[] built = new FakeHardwareMap[1];
        CameraConfig camera = config.camera();
        SceneFrameSource frames = new SceneFrameSource(
                scene(),
                new SimulatedCamera(camera.name(), NOMINAL_OPTICS, mountOf(camera)),
                new SceneFrameSource.PoseSource() {
                    @Override
                    public Pose2d pose() {
                        return built[0].drive().pose();
                    }
                });
        builder.addWebcam(camera.name(), frames, camera.framesPerSecond());

        FakeHardwareMap hardware = builder.withDrivetrain(config, field).build();
        built[0] = hardware;
        for (Map.Entry<String, MotorConfig> entry : config.motors().entrySet()) {
            MotorConfig motor = entry.getValue();
            hardware.motor(entry.getKey())
                    .state()
                    .setMaxSpeed(motor.rpm(), motor.ticksPerRevolution());
        }
        return hardware;
    }

    /**
     * What the camera is looking at.
     *
     * <p>Overridable so a subclass can decide at runtime; the scene it is handed defaults to the
     * official BioBuzz field, which needs no file to be correct.</p>
     */
    protected SimulatedScene scene() {
        return scene;
    }

    private static Pose3d mountOf(CameraConfig camera) {
        return Pose3d.ofDegrees(
                new Vec3(camera.forwardMetres(), camera.leftMetres(), camera.heightMetres()),
                camera.yawDegrees(), camera.pitchDegrees(), camera.rollDegrees());
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
