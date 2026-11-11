package org.firstinspires.ftc.teamcode.simulated;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.BlueLeftTipAware;
import org.firstinspires.ftc.teamcode.pedroPathing.BlueRightTipAware;
import org.firstinspires.ftc.teamcode.pedroPathing.RedLeftTipAware;
import org.firstinspires.ftc.teamcode.pedroPathing.RedRightTipAware;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.ngicollective.ftcsim.app.PlainJvmVision;
import org.ngicollective.ftcsim.camera.SimulatedScene;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.harness.LinearOpModeHarness;
import org.ngicollective.ftcsim.harness.OpModeHarness;
import org.ngicollective.ftcsim.physics.FieldPhysics;
import org.ngicollective.ftcsim.physics.PivotState;
import org.ngicollective.ftcsim.season.BioBuzzField;
import org.ngicollective.ftcsim.season.BioBuzzScenario;
import org.ngicollective.ftcsim.season.BioBuzzScore;
import org.ngicollective.ftcsim.sim.FieldConfig;
import org.ngicollective.ftcsim.sim.Pose2d;
import org.ngicollective.ftcsim.sim.RobotConfig;
import org.ngicollective.ftcsim.sim.SimConfigFiles;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

import java.util.ArrayList;
import java.util.List;

/**
 * That the tip-aware AUTOs read which way their HIVE faces and shoot from that side.
 *
 * <p>Each of the four starts, once on the match-start field and once with both HIVEs tipped
 * ({@code tipped-match-setup}). Real detector on rendered frames, real chassis and flywheel. The
 * assertions are why and where the robot stood when it first shot, which is the decision, and
 * that it parked inside the 30&nbsp;s AUTO.</p>
 */
@RunWith(RobolectricTestRunner.class)
@Config(sdk = 28)
public class TipAwareAutoAcceptanceTest {

    private static final double TICK_SECONDS = 0.02;
    private static final double METRES_PER_INCH = 0.0254;

    // Start squares in the field frame, from AllianceAuto's Pedro poses (pedro = field + 72 in).
    private static final Pose2d RED_FRONT = pedro(56.0, 9.5, 90.0);
    private static final Pose2d RED_REAR = pedro(62.0, 134.5, 270.0);
    private static final Pose2d BLUE_FRONT = pedro(88.0, 134.5, 270.0);
    private static final Pose2d BLUE_REAR = pedro(82.0, 9.5, 90.0);

    // Where each shoots from: the start it is on, or the other side's spot.
    private static final Pose2d RED_SHOOT = pedro(56.0, 10.5, 90.0);
    private static final Pose2d BLUE_SHOOT = pedro(88.0, 133.5, 270.0);

    /** How far from its shooting spot, in metres, the robot may be when it shoots. */
    private static final double SPOT_TOLERANCE = 0.08;

    @Test
    public void redLeftShootsFromTheAudienceSideWhenTheHiveIsUntipped() throws Exception {
        assertShootsFrom(new RedLeftTipAware(), RED_FRONT, "match-setup", true, "audience side",
                RED_FRONT);
    }

    @Test
    public void redLeftDrivesRoundToTheRearWhenTheHiveIsTipped() throws Exception {
        assertShootsFrom(new RedLeftTipAware(), RED_FRONT, "tipped-match-setup", true,
                "rear side", RED_REAR);
    }

    @Test
    public void redRightDrivesRoundToTheAudienceSideWhenTheHiveIsUntipped() throws Exception {
        assertShootsFrom(new RedRightTipAware(), RED_REAR, "match-setup", true, "audience side",
                RED_SHOOT);
    }

    @Test
    public void redRightShootsFromTheRearWhenTheHiveIsTipped() throws Exception {
        assertShootsFrom(new RedRightTipAware(), RED_REAR, "tipped-match-setup", true,
                "rear side", RED_REAR);
    }

    @Test
    public void blueLeftShootsFromTheRearWhenTheHiveIsUntipped() throws Exception {
        assertShootsFrom(new BlueLeftTipAware(), BLUE_FRONT, "match-setup", false, "rear side",
                BLUE_FRONT);
    }

    @Test
    public void blueLeftDrivesRoundToTheAudienceSideWhenTheHiveIsTipped() throws Exception {
        assertShootsFrom(new BlueLeftTipAware(), BLUE_FRONT, "tipped-match-setup", false,
                "audience side", BLUE_REAR);
    }

    @Test
    public void blueRightDrivesRoundToTheRearWhenTheHiveIsUntipped() throws Exception {
        assertShootsFrom(new BlueRightTipAware(), BLUE_REAR, "match-setup", false, "rear side",
                BLUE_SHOOT);
    }

    @Test
    public void blueRightShootsFromTheAudienceSideWhenTheHiveIsTipped() throws Exception {
        assertShootsFrom(new BlueRightTipAware(), BLUE_REAR, "tipped-match-setup", false,
                "audience side", BLUE_REAR);
    }

    private static void assertShootsFrom(LinearOpMode auto, Pose2d start, String scenarioName,
                                         boolean red, String side, Pose2d spot)
            throws Exception {
        PlainJvmVision.start();

        RobotConfig config = SimConfigFiles.robot("verity");
        BioBuzzScenario scenario = BioBuzzScenario.named(scenarioName);
        SimulatedScene scene = scenario.scene();
        FakeHardwareMap hardware = new VerityRobot(config, FieldConfig.standard(), scene).create();
        FieldPhysics world = FieldPhysics.of(scene.elements(), scene.structures(),
                FieldConfig.standard(), config);
        hardware.setPhysics(world);
        hardware.drive().setPose(start);
        int before = points(scenario, world, red);

        LinearOpModeHarness harness = OpModeHarness.forLinear(auto, hardware);
        harness.launch();
        List<String> timeline = new ArrayList<>();
        String firstShotStep = null;
        Pose2d firstShotPose = null;
        boolean done = false;
        double doneAtSeconds = Double.NaN;
        try {
            // A second and a half of INIT, for the camera to open and look.
            advance(harness, 1.5, timeline);
            harness.pressStart();
            for (int tick = 0; tick < 40.0 / TICK_SECONDS && !done; tick++) {
                List<String> frame = step(harness, timeline);
                String now = value(frame, "step");
                if (firstShotStep == null && now != null && now.startsWith("shooting")) {
                    firstShotStep = value(frame, "camera");
                    firstShotPose = hardware.drive().pose();
                }
                done = "done".equals(now);
                if (done) {
                    doneAtSeconds = seconds(value(frame, "time"));
                }
            }
        } finally {
            harness.pressStop();
            harness.awaitCompletion(5000);
        }
        System.out.println(auto.getClass().getSimpleName() + " on " + scenarioName + ":\n  "
                + String.join("\n  ", timeline));

        assertTrue("the routine should have ended itself; it went " + timeline, done);
        assertTrue("it never shot; it went " + timeline, firstShotPose != null);
        assertTrue("it should have shot from the " + side + ", but shot because " + firstShotStep
                        + " at " + firstShotPose,
                firstShotStep.endsWith("from the " + side) || firstShotStep.endsWith("at START"));
        assertEquals("first shot from " + spot + ", but it was at " + firstShotPose, 0.0,
                Math.hypot(firstShotPose.x() - spot.x(), firstShotPose.y() - spot.y()),
                SPOT_TOLERANCE);

        assertOneCollectionPerShot(timeline);
        assertTrue("it should have parked inside the 30 s AUTO; it went " + timeline,
                doneAtSeconds <= 30.0);
        // Points are printed, not asserted: the simulated robot has no pre-loads and no staging,
        // and its intake does not reach the GARDEN's line of POLLEN, so this routine has nothing
        // to score in the sim. Where it stood to shoot is the decision under test.
        System.out.println("  points " + before + " -> " + points(scenario, world, red));
    }

    /** Every trip to the GARDEN or the FLOWER is followed by a shot before the next trip. */
    private static void assertOneCollectionPerShot(List<String> timeline) {
        boolean carrying = false;
        for (String line : timeline) {
            boolean collecting = line.startsWith("sweeping") || line.startsWith("collecting");
            assertTrue("it went collecting twice without shooting in between: " + timeline,
                    !(collecting && carrying));
            if (collecting) {
                carrying = true;
            } else if (line.startsWith("shooting")) {
                carrying = false;
            }
        }
    }

    /** "12.3 s" to 12.3. */
    private static double seconds(String time) {
        return time == null ? Double.NaN : Double.parseDouble(time.replace("s", "").trim());
    }

    private static int points(BioBuzzScenario scenario, FieldPhysics world, boolean red) {
        BioBuzzScore score = BioBuzzScore.of(
                BioBuzzField.scene(scenario.redHive(), scenario.blueHive())
                        .tipped(PivotState.anglesOf(world.pivots())).scoringVolumes(),
                world.elements(), PivotState.swingsOf(world.pivots()));
        return red ? score.redPoints() : score.bluePoints();
    }

    private static void advance(LinearOpModeHarness harness, double seconds, List<String> timeline)
            throws InterruptedException {
        for (int tick = 0; tick < seconds / TICK_SECONDS; tick++) {
            step(harness, timeline);
        }
    }

    /** One tick, paced against the wall clock the OpMode times itself by. */
    private static List<String> step(LinearOpModeHarness harness, List<String> timeline)
            throws InterruptedException {
        harness.advance(TICK_SECONDS);
        PlainJvmVision.pump();
        Thread.sleep(20);
        List<String> frame = harness.lastTelemetry();
        String now = value(frame, "step");
        String line = now + " | " + value(frame, "camera") + " | " + value(frame, "time")
                + " | " + value(frame, "pose");
        if (now != null && (timeline.isEmpty()
                || !timeline.get(timeline.size() - 1).startsWith(now + " |"))) {
            timeline.add(line);
        }
        return frame;
    }

    /** The value of a "caption : value" telemetry line, or null. */
    private static String value(List<String> frame, String caption) {
        for (String line : frame) {
            int colon = line.indexOf(':');
            if (colon > 0 && line.substring(0, colon).trim().equals(caption)) {
                return line.substring(colon + 1).trim();
            }
        }
        return null;
    }

    private static Pose2d pedro(double x, double y, double headingDegrees) {
        return new Pose2d((x - 72.0) * METRES_PER_INCH, (y - 72.0) * METRES_PER_INCH,
                Math.toRadians(headingDegrees));
    }
}
