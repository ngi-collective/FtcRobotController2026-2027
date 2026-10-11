package org.firstinspires.ftc.teamcode.simulated;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.AimedLauncherTeleOp;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;
import org.ngicollective.testframework.camera.GameElement;
import org.ngicollective.testframework.camera.SimulatedScene;
import org.ngicollective.testframework.camera.TagCluster;
import org.ngicollective.testframework.camera.Vec3;
import org.ngicollective.testframework.harness.LinearOpModeHarness;
import org.ngicollective.testframework.harness.OpModeHarness;
import org.ngicollective.testframework.hardware.FakeHardwareMap;
import org.ngicollective.testframework.physics.FieldPhysics;
import org.ngicollective.testframework.physics.PivotState;
import org.ngicollective.testframework.season.BioBuzzField;
import org.ngicollective.testframework.season.BioBuzzHive;
import org.ngicollective.testframework.sim.FieldConfig;
import org.ngicollective.testframework.sim.Pose2d;
import org.ngicollective.testframework.sim.RobotConfig;
import org.ngicollective.testframework.sim.SimConfigFiles;
import org.ngicollective.testframework.sim.VolumeConfig;

import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * The season's loop, end to end: see a CELL, range off it, shoot, and watch the tags swap.
 *
 * <p>Everything in it is the real article. The detector is the SDK's own
 * {@code AprilTagProcessor} running on real OpenCV; the frames are rendered from the simulated
 * field through the lens the solver inverts; the OpMode is the one a driver would run, driven only
 * through its gamepad; the flywheel has inertia and the HIVE is on a hinge. Nothing here tells the
 * world that a shot happened or that a HIVE should tip.</p>
 *
 * <p>Runs on the plain JVM execution target, and so in CI, which it could not do while the SDK's
 * vision natives were the only ones available: see {@link PlainJvmVision} and
 * {@code docs/adr/0007-vision-runs-on-the-plain-jvm.md}. Each test class gets its own Robolectric
 * sandbox, which is also what keeps its {@code VisionPortal} apart from the one
 * {@code SyntheticCameraAcceptanceTest} builds &mdash; two LiveView portals in one classloader
 * fail on the viewport neither can get back.</p>
 */
@RunWith(RobolectricTestRunner.class)
@Config(sdk = 28)
public class AimedLauncherAcceptanceTest {

    /** "target : RED AUDIENCE  100% of the cluster", as the OpMode prints it. */
    private static final Pattern TARGET = Pattern.compile("target\\s*:\\s*(\\S+ \\S+)\\s+(\\d+)%");

    /** "target : no RED CELL in view" reads as a target line too, so it needs its own pattern. */
    private static final Pattern TARGET_LOST = Pattern.compile("target\\s*:\\s*(no .+)");

    /** "range : 0.98 m to a mouth 1.51 m up". */
    private static final Pattern RANGE =
            Pattern.compile("range\\s*:\\s*(-?[\\d.]+) m to a mouth (-?[\\d.]+) m up");

    /** "solved : 2032 rpm", or the reason there is no shot. */
    private static final Pattern SOLVED = Pattern.compile("solved\\s*:\\s*(\\d+) rpm");

    /** "state : READY" and its several ways of not being ready. */
    private static final Pattern STATE = Pattern.compile("state\\s*:\\s*(.+)");

    /** Where the scenario lays its row of POLLEN out, in the field frame. */
    private static final double POLLEN_ROW_Y = -1.26;

    /** Square on to the red HIVE, which hangs on the -X side of the field's centre line. */
    private static final double SHOT_FROM_X = -0.324;

    /**
     * How much further back than the mouth's own reach the robot spins up, in metres.
     *
     * <p>Small on purpose, and this is the crux of the choreography. On this robot the mouth is
     * the wheel: a ball comes within reach and goes, so a full-speed shot means spinning up first
     * and collecting second (the same two steps {@code LauncherTest.spunUpOver} takes, and for the
     * same reason). But the speed was solved for the range the robot spun up at, and a steep lob
     * overshoots by about whatever it then moved forward &mdash; against a CELL opening whose lips
     * are only 9&nbsp;cm apart in the direction of flight. Two centimetres of roll is what a
     * driver's last nudge is worth and leaves the ball comfortably inside.</p>
     *
     * <p>Which is also the finding: <b>this robot cannot shoot on the move.</b> At a third of
     * power the chassis adds 0.55&nbsp;m/s to a 1.45&nbsp;m/s horizontal component and the ball
     * sails over the far lip. Anything that wants to fire while driving needs a feeder holding
     * balls back from the wheel, which this robot has not got.</p>
     */
    private static final double CREEP_METRES = 0.02;

    private static final double TICK_SECONDS = 0.02;

    /**
     * Where the robot spins up: far enough back that the POLLEN is just outside the mouth's reach.
     *
     * <p>Derived from the configuration rather than written down, because "just outside" is a
     * relation between the mouth's length and a ball's radius, and a mouth someone widens would
     * otherwise quietly start the test with three balls already in the wheel.</p>
     */
    private static Pose2d firingSpot(RobotConfig config) {
        return new Pose2d(SHOT_FROM_X, POLLEN_ROW_Y - reachMetres(config) - 0.004,
                Math.toRadians(90.0));
    }

    /** How far ahead of the robot's centre a ball's surface first touches the mouth. */
    private static double reachMetres(RobotConfig config) {
        VolumeConfig mouth = config.launchers().get("flywheel").mouth();
        return mouth.forwardMetres() + mouth.lengthMetres() / 2.0
                + GameElement.POLLEN_DIAMETER_METRES / 2.0;
    }

    @Test
    public void theRobotRangesOffAClusterAndTipsTheHiveItWasAimingAt() throws Exception {
        PlainJvmVision.start();

        RobotConfig config = SimConfigFiles.robot("verity");
        SimulatedScene scene = SimConfigFiles.scenario("aimed-shot").scene();
        FakeHardwareMap hardware = new VerityRobot(config, FieldConfig.standard(), scene).create();

        // The world has to be installed, not merely rendered. A hardware map is built with an
        // empty field on purpose -- see FakeHardwareMap.Builder -- and a dashboard session fills
        // it at every INIT; a test that skipped this would have a camera looking at a scene full
        // of balls and structures with nothing solid in front of the robot at all, which reads as
        // a launcher that fires into a field that ignores it.
        hardware.setPhysics(FieldPhysics.of(scene.elements(), scene.structures(),
                FieldConfig.standard(), config));
        hardware.drive().setPose(firingSpot(config));

        LinearOpModeHarness harness =
                OpModeHarness.forLinear(new AimedLauncherTeleOp(), hardware);
        harness.launch();
        harness.pressStart();

        // Hold the assist and nothing else: no ball is in the mouth yet, so this is the OpMode
        // turning onto the CELL and spinning up on its own.
        harness.gamepad1().left_bumper = true;
        List<String> aimed = advanceUntil(harness, 6.0, frame -> matched(frame, STATE, "READY"));
        System.out.println("aimed: " + aimed);

        // What the camera worked out, against what the scene was built from. This is the whole
        // chain in one assertion: the renderer's geometry, the detector's solve, the camera's
        // 35-degree tilt taken back out, and the muzzle's own reach ahead of the chassis.
        Matcher range = find(aimed, RANGE);
        double reportedDistance = Double.parseDouble(range.group(1));
        double reportedHeight = Double.parseDouble(range.group(2));

        Vec3 origin = clusterOrigin(BioBuzzField.RED_AUDIENCE);
        double trueDistance = Math.abs(origin.y()
                - (firingSpot(config).y() + muzzleForwardMetres()));
        assertEquals("the range the OpMode ranged off the tags, against the field's own geometry",
                trueDistance, reportedDistance, 0.05);
        assertEquals("and the height of the opening it is aiming into",
                origin.z(), reportedHeight, 0.05);

        Matcher target = find(aimed, TARGET);
        assertEquals("the raised CELL is the one on the audience side of an audience-up HIVE",
                BioBuzzField.RED_AUDIENCE, target.group(1));
        assertEquals("all four tags of the cluster should be in a 1 m view",
                100, Integer.parseInt(target.group(2)));

        // A steep lob from a metre out: a few hundred rev/min either side of the figure the
        // simulator was tuned to by hand, and nowhere near the wheel's 6000.
        int solvedRpm = Integer.parseInt(find(aimed, SOLVED).group(1));
        assertTrue("implausible wheel speed " + solvedRpm + " rpm", solvedRpm > 1500
                && solvedRpm < 2600);

        // Now collect, by rolling forward onto the POLLEN two centimetres at a time. Three balls
        // fit across the mouth and leave together, so each nudge is a three-ball burst at the
        // speed solved for where the robot was standing when it took it -- which is what a driver
        // does, and the only thing this robot can do: see CREEP_METRES.
        int bursts = 0;
        for (double rolled = CREEP_METRES; rolled < 0.30 && !tipped(hardware);
                rolled += CREEP_METRES) {
            hardware.drive().setPose(new Pose2d(SHOT_FROM_X, firingSpot(config).y() + rolled,
                    Math.toRadians(90.0)));
            // Long enough for a burst to fly and for the wheel to settle on the speed the new
            // range wants, which is a few rev/min away: a 2 cm roll barely changes a steep lob.
            advanceUntil(harness, 1.0, frame -> tipped(hardware));
            bursts++;
        }

        assertTrue("the red HIVE never tipped after " + bursts + " nudges forward: the balls left"
                        + " the mouth but what arrived in the CELL was not enough to move it, or it"
                        + " did not arrive at all. Elements: " + hardware.physics().elements(),
                tipped(hardware));
        System.out.println("tipped after " + bursts + " nudges: " + hardware.physics().pivots());

        // And the payoff: a HIVE that has tipped faces the other way, so the robot that just
        // scored is looking at the backs of two plates and has no target at all. This is the
        // season's own consequence rather than a limitation -- the CELL worth shooting at is now
        // the other one, with the other four ids, on the far side of the field.
        List<String> lost = advanceUntil(harness, 4.0,
                frame -> matched(frame, TARGET_LOST, "no RED CELL in view"));
        System.out.println("after the tip: " + lost);
        assertTrue("the tipped HIVE's tags face away, so there should be nothing to aim at: "
                        + lost,
                matched(lost, TARGET_LOST, "no RED CELL in view"));

        // Drive round. The newly raised CELL is RED SCORING, and it ranges just like the other one
        // did: the loop closes again on a target that did not exist a moment ago.
        hardware.drive().setPose(new Pose2d(SHOT_FROM_X, -firingSpot(config).y(),
                Math.toRadians(-90.0)));
        List<String> again = advanceUntil(harness, 6.0,
                frame -> matched(frame, TARGET, BioBuzzField.RED_SCORING));
        System.out.println("from the scoring side: " + again);
        assertTrue("the other CELL should be the raised one now: " + again,
                matched(again, TARGET, BioBuzzField.RED_SCORING));

        Matcher swapped = find(again, RANGE);
        Vec3 scoring = clusterOrigin(BioBuzzField.RED_SCORING);
        assertEquals("and it should range the same as the first one did, mirrored",
                trueDistance, Double.parseDouble(swapped.group(1)), 0.05);
        assertEquals("at the height a raised CELL's opening sits at",
                scoring.z(), Double.parseDouble(swapped.group(2)), 0.05);

        harness.pressStop();
        harness.awaitCompletion(5000);
    }

    /**
     * Where the tags the camera is looking at actually are, in the state the HIVE is in.
     *
     * <p>The cluster's origin rather than the centre of the CELL opening, because the origin is
     * what a detection reports and therefore what the OpMode's range is a range to. The two are an
     * inch and a half apart, which {@code LaunchGeometryTest} is the place to care about.</p>
     */
    private static Vec3 clusterOrigin(String cell) {
        boolean audienceUp = BioBuzzField.RED_AUDIENCE.equals(cell);
        BioBuzzField.HiveTip tip = audienceUp
                ? BioBuzzField.HiveTip.AUDIENCE_UP : BioBuzzField.HiveTip.AUDIENCE_DOWN;
        for (TagCluster cluster : BioBuzzField.clusters(tip, tip)) {
            if (cluster.name().equals(cell)) {
                return cluster.pose().position();
            }
        }
        throw new AssertionError("no cluster called " + cell);
    }

    /** How far ahead of the robot's centre a ball leaves, from the configuration. */
    private static double muzzleForwardMetres() {
        return SimConfigFiles.robot("verity").launchers().get("flywheel").mouth().forwardMetres();
    }

    /** Whether the red HIVE has swung all the way to its other stop. */
    private static boolean tipped(FakeHardwareMap hardware) {
        for (PivotState pivot : hardware.physics().pivots()) {
            if (pivot.structureName().equals(BioBuzzHive.RED) && pivot.completedSwings() > 0) {
                return true;
            }
        }
        return false;
    }

    /**
     * Steps the simulation until a telemetry frame satisfies the test, and hands that frame back.
     *
     * <p>Real time as well as simulated: the frames have to be rendered and the detector has to
     * run, and that work happens on a CPU whatever the clock says.</p>
     */
    private static List<String> advanceUntil(LinearOpModeHarness harness, double seconds,
                                             FramePredicate wanted) throws InterruptedException {
        List<String> last = harness.lastTelemetry();
        for (int tick = 0; tick < seconds / TICK_SECONDS; tick++) {
            harness.advance(TICK_SECONDS);
            PlainJvmVision.pump();
            Thread.sleep(8);
            last = harness.lastTelemetry();
            if (wanted.test(last)) {
                return last;
            }
        }
        return last;
    }

    private static boolean matched(List<String> frame, Pattern pattern, String expected) {
        for (String line : frame) {
            Matcher matcher = pattern.matcher(line);
            if (matcher.find() && matcher.group(1).startsWith(expected)) {
                return true;
            }
        }
        return false;
    }

    private static Matcher find(List<String> frame, Pattern pattern) {
        for (String line : frame) {
            Matcher matcher = pattern.matcher(line);
            if (matcher.find()) {
                return matcher;
            }
        }
        assertNotNull("no telemetry line matched " + pattern + " in " + frame, null);
        throw new AssertionError();
    }

    private interface FramePredicate {
        boolean test(List<String> frame);
    }
}
