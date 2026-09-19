package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.simulated.VerityRobot;

import org.junit.jupiter.api.Test;
import org.ngicollective.testframework.camera.GameElement;
import org.ngicollective.testframework.camera.SimulatedScene;
import org.ngicollective.testframework.harness.LinearOpModeHarness;
import org.ngicollective.testframework.harness.OpModeHarness;
import org.ngicollective.testframework.hardware.FakeHardwareMap;
import org.ngicollective.testframework.physics.FieldPhysics;
import org.ngicollective.testframework.physics.PivotState;
import org.ngicollective.testframework.season.BioBuzzField;
import org.ngicollective.testframework.season.BioBuzzHive;
import org.ngicollective.testframework.season.BioBuzzScore;
import org.ngicollective.testframework.sim.FieldConfig;
import org.ngicollective.testframework.sim.Pose2d;
import org.ngicollective.testframework.sim.RobotConfig;
import org.ngicollective.testframework.sim.SimConfigFiles;

import java.util.Collections;
import java.util.List;

/**
 * That the AUTO actually scores, run against the field it is written for.
 *
 * <p>On a plain JVM, and that is the point of an AUTO that aims off the field drawing rather than
 * off a camera: there is no detector in this process and none is needed, so the routine a team
 * runs first is also the one that can be regression-tested on every commit. The vision loop's
 * equivalent needs an emulator and sits outside CI.</p>
 *
 * <p>Nothing here is mocked. The flywheel has inertia, the balls fly under gravity, the basket is
 * the manual's dimensions and the HIVE is on a hinge. The test presses START and reads the score.</p>
 */
class SweepAndShootAutoTest {

    private static final BioBuzzField.HiveTip RED_UP = BioBuzzField.HiveTip.AUDIENCE_UP;
    private static final BioBuzzField.HiveTip BLUE_DOWN = BioBuzzField.HiveTip.AUDIENCE_DOWN;

    /** The square the OpMode's own INIT telemetry names, which is what a driver lines up on. */
    private static final Pose2d START = new Pose2d(-0.55, -1.431, Math.toRadians(90.0));

    /**
     * Simulated seconds per step, paced against the wall clock.
     *
     * <p>An OpMode's own waits are on the wall clock, because that is the only clock a real robot
     * has, so a test that ran the world faster than real time would have the routine's steps end
     * early and the sweep cover a fraction of the row. Stepping 20&nbsp;ms of world per 20&nbsp;ms
     * of wall clock is what keeps the two agreeing.</p>
     */
    private static final double TICK = 0.02;

    @Test
    void theAutoSweepsSixPollenIntoTheRaisedCellAndTipsIt() throws InterruptedException {
        RobotConfig config = SimConfigFiles.robot("verity");
        SimulatedScene scene = SimConfigFiles.scenario("auto-sweep").scene();
        FakeHardwareMap hardware = new VerityRobot(config, FieldConfig.standard(), scene).create();
        FieldPhysics world = FieldPhysics.of(scene.elements(), scene.structures(),
                FieldConfig.standard(), config);
        hardware.setPhysics(world);
        hardware.drive().setPose(START);

        LinearOpModeHarness harness =
                OpModeHarness.forLinear(new SweepAndShootAuto(), hardware);
        harness.launch();
        harness.pressStart();

        // The routine is about 7 s of spin-up, two sweeps and a settle, and then it stands there
        // reporting "done" until something stops it -- which is the point, since a dashboard
        // session rebuilds the field the moment an OpMode ends.
        List<String> telemetry = Collections.emptyList();
        for (int tick = 0; tick < 500 && !done(telemetry); tick++) {
            harness.advance(TICK);
            Thread.sleep(20);
            telemetry = harness.lastTelemetry();
        }

        assertTrue(done(telemetry),
                "the routine should have reported itself done; its last word was " + telemetry);

        PivotState hive = world.pivots().get(0);
        assertEquals(BioBuzzHive.RED, hive.structureName());
        assertEquals(1, hive.completedSwings(),
                "six POLLEN swept into a CELL already holding three NECTAR should tip the red"
                        + " HIVE; it is at " + Math.toDegrees(hive.angleRadians()) + " degrees, and"
                        + " the balls ended up at " + world.elements());

        // Scored against the CELLs where they are now, which after a tip is not where they were:
        // the TIP is worth more than everything that caused it, and the basket has emptied itself.
        BioBuzzScore score = BioBuzzScore.of(
                BioBuzzField.scene(RED_UP, BLUE_DOWN).tipped(PivotState.anglesOf(world.pivots()))
                        .scoringVolumes(),
                world.elements(), PivotState.swingsOf(world.pivots()));
        assertEquals(BioBuzzScore.POINTS_PER_TIP, score.redPoints(),
                "twenty for the TIP: " + score);
        assertEquals(0, score.bluePoints(), "and nothing for the other alliance: " + score);

        harness.pressStop();
        harness.awaitCompletion(2000);
    }

    /** Whether the routine has reported itself finished, which is its last telemetry line. */
    private static boolean done(List<String> telemetry) {
        for (String line : telemetry) {
            if (line.contains("done")) {
                return true;
            }
        }
        return false;
    }

    /**
     * And that a robot lined up on the wrong square throws its POLLEN on the floor.
     *
     * <p>The interesting failure of a timed AUTO, and the price of aiming off a drawing. Half a
     * metre back, the shot the routine takes is still the one it solved for the square it was told
     * it would be on, so every ball falls short and lands on the tiles. The routine cannot tell,
     * which is exactly what a team sees on the day when they set the robot down by eye, and the
     * point of the assertion is that the simulator does not paper over it.</p>
     *
     * <p>Stopped after the first sweep rather than run to the end: the balls are already on the
     * floor by then and the remaining six seconds would only be six seconds.</p>
     */
    @Test
    void offItsStartSquareTheSameRoutineThrowsThePollenOnTheFloor() throws InterruptedException {
        RobotConfig config = SimConfigFiles.robot("verity");
        SimulatedScene scene = SimConfigFiles.scenario("auto-sweep").scene();
        FakeHardwareMap hardware = new VerityRobot(config, FieldConfig.standard(), scene).create();
        FieldPhysics world = FieldPhysics.of(scene.elements(), scene.structures(),
                FieldConfig.standard(), config);
        hardware.setPhysics(world);
        hardware.drive().setPose(new Pose2d(START.x(), START.y() - 0.5, START.heading()));

        LinearOpModeHarness harness = OpModeHarness.forLinear(new SweepAndShootAuto(), hardware);
        harness.launch();
        harness.pressStart();
        for (int tick = 0; tick < 250 && !harness.isFinished(); tick++) {
            harness.advance(TICK);
            Thread.sleep(20);
        }

        int inTheCell = 0;
        for (GameElement element : world.elements()) {
            if (element.centre().z() > 1.0) {
                inTheCell++;
            }
        }
        assertEquals(3, inTheCell,
                "only the three staged NECTAR should be up in the CELL; the swept POLLEN fell"
                        + " short and should be lying on the tiles: " + world.elements());
        assertEquals(0, world.pivots().get(0).completedSwings(),
                "and nothing should have tipped");

        harness.pressStop();
        harness.awaitCompletion(2000);
    }
}
