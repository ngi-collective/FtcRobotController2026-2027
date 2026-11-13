package org.firstinspires.ftc.teamcode.simulated;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoTimeline;
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

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Runs one tip-aware AUTO and checks why and where it first shot, then {@link AutoTimeline}'s rules.
 *
 * <p>Real detector on rendered frames, real chassis and flywheel, paced against the wall clock.
 * Called from Robolectric test classes, two runs to a class, so Gradle's parallel forks can run
 * them side by side; each class gets its own sandbox and so its own LiveView.</p>
 */
final class TipAwareAutoRun {

    private static final double TICK_SECONDS = 0.02;
    private static final double METRES_PER_INCH = 0.0254;

    // Start squares in the field frame, from AllianceAuto's Pedro poses (pedro = field + 72 in).
    static final Pose2d RED_FRONT = pedro(56.0, 9.5, 90.0);
    static final Pose2d RED_REAR = pedro(62.0, 134.5, 270.0);
    static final Pose2d BLUE_FRONT = pedro(88.0, 134.5, 270.0);
    static final Pose2d BLUE_REAR = pedro(82.0, 9.5, 90.0);

    // Where each shoots from: the start it is on, or the other side's spot.
    static final Pose2d RED_SHOOT = pedro(56.0, 10.5, 90.0);
    static final Pose2d BLUE_SHOOT = pedro(88.0, 133.5, 270.0);

    /** How far from its shooting spot, in metres, the robot may be when it shoots. */
    private static final double SPOT_TOLERANCE = 0.08;

    /**
     * The simulated camera's frame rate in these runs, against the 30 the robot's file says.
     *
     * <p>Frames are drawn in software, on the same CPU the OpMode's follower loop and the other
     * parallel test forks need, and at 30 a second that starved them: parks stalled half-way and
     * routines overran 30 s. The decision does not need them. {@code RaisedCellWatch} judges over
     * a half-second window, which still holds five frames.</p>
     */
    private static final double CAMERA_FRAMES_PER_SECOND = 10.0;

    private TipAwareAutoRun() {
    }

    /** {@code verity.json} with the camera at {@link #CAMERA_FRAMES_PER_SECOND}. */
    private static RobotConfig verity() throws Exception {
        Path original = SimConfigFiles.directory().resolve("verity.json");
        String json = new String(Files.readAllBytes(original), StandardCharsets.UTF_8);
        String slower = json.replaceFirst("\"framesPerSecond\"\\s*:\\s*[0-9.]+",
                "\"framesPerSecond\": " + CAMERA_FRAMES_PER_SECOND);
        if (slower.equals(json)) {
            throw new IllegalStateException("no camera framesPerSecond in " + original);
        }
        Path directory = Files.createTempDirectory("tip-aware-robot");
        Files.write(directory.resolve("verity.json"), slower.getBytes(StandardCharsets.UTF_8));
        return SimConfigFiles.robot(directory, "verity");
    }

    static void assertShootsFrom(LinearOpMode auto, Pose2d start, String scenarioName,
                                         boolean red, String side, Pose2d spot)
            throws Exception {
        PlainJvmVision.start();

        RobotConfig config = verity();
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
        AutoTimeline mechanisms = null;
        try {
            // A second and a half of INIT, for the camera to open and look.
            advance(harness, 1.5, timeline);
            harness.pressStart();
            mechanisms = new AutoTimeline(hardware);
            for (int tick = 0; tick < 40.0 / TICK_SECONDS && !done; tick++) {
                List<String> frame = step(harness, timeline);
                String now = value(frame, "step");
                mechanisms.sample(now == null ? "" : now);
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

        System.out.println("  " + mechanisms.timeline());
        mechanisms.assertKeptTheRules(auto.getClass().getSimpleName() + " on " + scenarioName,
                doneAtSeconds, red);
        // Points are printed, not asserted: the simulated robot has no pre-loads and no staging,
        // and its intake does not reach the GARDEN's line of POLLEN, so this routine has nothing
        // to score in the sim. Where it stood to shoot is the decision under test.
        System.out.println("  points " + before + " -> " + points(scenario, world, red));
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
