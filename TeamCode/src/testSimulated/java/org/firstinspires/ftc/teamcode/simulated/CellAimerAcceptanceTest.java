package org.firstinspires.ftc.teamcode.simulated;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CellAimer;
import org.firstinspires.ftc.teamcode.DriveHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.ngicollective.ftcsim.app.PlainJvmVision;
import org.ngicollective.ftcsim.camera.SimulatedScene;
import org.ngicollective.ftcsim.camera.TagCluster;
import org.ngicollective.ftcsim.camera.Vec3;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.harness.LinearOpModeHarness;
import org.ngicollective.ftcsim.harness.OpModeHarness;
import org.ngicollective.ftcsim.physics.FieldPhysics;
import org.ngicollective.ftcsim.season.BioBuzzField;
import org.ngicollective.ftcsim.season.BioBuzzScenario;
import org.ngicollective.ftcsim.sim.FieldConfig;
import org.ngicollective.ftcsim.sim.Pose2d;
import org.ngicollective.ftcsim.sim.RobotConfig;
import org.ngicollective.ftcsim.sim.SimConfigFiles;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

import java.util.List;

/**
 * That {@link CellAimer} turns the robot onto the raised CELL, and gives up when there is none.
 *
 * <p>Asserted on the robot's pose, not on the wheel powers it asked for: what the driver cares
 * about is where the nose ends up, and a power with the wrong sign is only visible as a heading
 * that walks away. Real detector, real frames, real chassis inertia.</p>
 *
 * <p>Both cases run in one OpMode because each Robolectric test class gets one {@code VisionPortal}
 * worth of LiveView: see {@code AimedLauncherAcceptanceTest}.</p>
 */
@RunWith(RobolectricTestRunner.class)
@Config(sdk = 28)
public class CellAimerAcceptanceTest {

    private static final double TICK_SECONDS = 0.02;

    /**
     * Close under the red HIVE's raised CELL, on its centre line.
     *
     * <p>Close because the camera points straight up and sees the cluster only from about half a
     * metre short of it; this is inside that band, and turning in place keeps it there.</p>
     */
    private static final double START_X = -0.324;
    private static final double START_Y = -0.85;

    /** Aims twice: once at nothing, and again after the test presses A. */
    public static class AimTwice extends LinearOpMode {

        @Override
        public void runOpMode() {
            DriveHardware drive = DriveHardware.initWithoutEncoders(hardwareMap);
            AprilTagProcessor tags = new AprilTagProcessor.Builder()
                    .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                    .build();
            VisionPortal camera = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(tags)
                    .build();
            waitForStart();

            CellAimer aimer = new CellAimer(tags, drive, true);
            boolean first = aimer.aim(this, 30.0);
            while (opModeIsActive() && !gamepad1.a) {
                telemetry.addData("first", first);
                telemetry.update();
            }
            boolean second = aimer.aim(this, 30.0);
            while (opModeIsActive()) {
                telemetry.addData("first", first);
                telemetry.addData("second", second);
                telemetry.update();
            }
            camera.close();
        }
    }

    @Test
    public void turnsOntoTheRaisedCellAndGivesUpWhenThereIsNone() throws Exception {
        PlainJvmVision.start();

        RobotConfig config = SimConfigFiles.robot("verity");
        SimulatedScene scene = BioBuzzScenario.named("aimed-shot").scene();
        FakeHardwareMap hardware = new VerityRobot(config, FieldConfig.standard(), scene).create();
        hardware.setPhysics(FieldPhysics.of(scene.elements(), scene.structures(),
                FieldConfig.standard(), config));

        // Nose at the far wall: the HIVE is behind the robot and the camera sees only ceiling.
        hardware.drive().setPose(new Pose2d(START_X, START_Y, Math.toRadians(-90.0)));

        LinearOpModeHarness harness = OpModeHarness.forLinear(new AimTwice(), hardware);
        harness.launch();
        harness.pressStart();
        try {
            aimAtNothingThenAtTheCell(harness, hardware);
        } finally {
            // Always, so that a failure here does not leave a VisionPortal open for the next
            // vision test in this JVM to collide with.
            harness.pressStop();
            harness.awaitCompletion(5000);
        }
    }

    private static void aimAtNothingThenAtTheCell(LinearOpModeHarness harness,
                                                  FakeHardwareMap hardware)
            throws InterruptedException {
        List<String> frame = advanceUntil(harness, 5.0, "first");
        assertTrue("with no CELL in view the aimer should give up, not spin: " + frame,
                has(frame, "first : false"));
        assertEquals("and it should have left the robot where it was",
                -90.0, Math.toDegrees(hardware.drive().pose().heading()), 2.0);

        // Fifteen degrees clockwise of the CELL: a counter-clockwise turn to make.
        hardware.drive().setPose(new Pose2d(START_X, START_Y, Math.toRadians(75.0)));
        harness.gamepad1().a = true;

        frame = advanceUntil(harness, 10.0, "second");
        assertTrue("the aimer should have settled on the CELL: " + frame,
                has(frame, "second : true"));

        Pose2d pose = hardware.drive().pose();
        Vec3 cell = clusterOrigin(BioBuzzField.RED_AUDIENCE);
        double bearing = Math.atan2(cell.y() - pose.y(), cell.x() - pose.x());
        assertEquals("the nose should point at the raised CELL",
                Math.toDegrees(bearing), Math.toDegrees(pose.heading()), CellAimer.READY_DEGREES);
    }

    private static Vec3 clusterOrigin(String cell) {
        for (TagCluster cluster : BioBuzzField.clusters(BioBuzzField.HiveTip.AUDIENCE_UP,
                BioBuzzField.HiveTip.AUDIENCE_DOWN)) {
            if (cluster.name().equals(cell)) {
                return cluster.pose().position();
            }
        }
        throw new AssertionError("no cluster called " + cell);
    }

    /**
     * Steps the world until a caption appears, for up to that many simulated seconds.
     *
     * <p>The aimer's timeouts are wall-clock and generous here, because rendering and detection
     * run slower than real time; what the test bounds is simulated time.</p>
     */
    private static List<String> advanceUntil(LinearOpModeHarness harness, double seconds,
                                             String caption) throws InterruptedException {
        List<String> last = harness.lastTelemetry();
        for (int tick = 0; tick < seconds / TICK_SECONDS; tick++) {
            harness.advance(TICK_SECONDS);
            PlainJvmVision.pump();
            Thread.sleep(20);
            last = harness.lastTelemetry();
            if (has(last, caption + " :")) {
                return last;
            }
        }
        return last;
    }

    private static boolean has(List<String> frame, String text) {
        for (String line : frame) {
            if (line.replaceAll("\\s+", " ").contains(text)) {
                return true;
            }
        }
        return false;
    }
}
