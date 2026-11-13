package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.ngicollective.ftcsim.camera.SimulatedScene;
import org.ngicollective.ftcsim.hardware.FakeHardwareMap;
import org.ngicollective.ftcsim.harness.LinearOpModeHarness;
import org.ngicollective.ftcsim.harness.OpModeHarness;
import org.ngicollective.ftcsim.physics.FieldPhysics;
import org.ngicollective.ftcsim.season.BioBuzzScenario;
import org.ngicollective.ftcsim.sim.FieldConfig;
import org.ngicollective.ftcsim.sim.Pose2d;
import org.ngicollective.ftcsim.sim.RobotConfig;
import org.ngicollective.ftcsim.sim.SimConfigFiles;

/**
 * Runs one {@code AllianceAuto} on {@code match-setup}, set down on its own start square, and
 * checks it shoots, intakes and parks in the right order and in time ({@link AutoTimeline}).
 *
 * <p>Paced against the wall clock, because the OpMode times its shots and waits by it, so a run
 * takes as long as the AUTO does. The tests are split two to a class so that Gradle's parallel
 * forks can run them side by side.</p>
 */
final class AllianceAutoRun {

    private static final double TICK = 0.02;

    private AllianceAutoRun() {
    }

    /** Runs one AUTO from a start square in Pedro inches and checks its timeline. */
    static void run(LinearOpMode auto, double x, double y, double headingDegrees,
                            boolean red) throws Exception {
        RobotConfig config = SimConfigFiles.robot("verity");
        SimulatedScene scene = BioBuzzScenario.named("match-setup").scene();
        FakeHardwareMap hardware = new VerityRobot(config, FieldConfig.standard(), scene).create();
        hardware.setPhysics(FieldPhysics.of(scene.elements(), scene.structures(),
                FieldConfig.standard(), config));
        hardware.drive().setPose(new Pose2d((x - 72) * 0.0254, (y - 72) * 0.0254,
                Math.toRadians(headingDegrees)));

        LinearOpModeHarness harness = OpModeHarness.forLinear(auto, hardware);
        harness.launch();
        harness.pressStart();
        AutoTimeline timeline = new AutoTimeline(hardware);
        double doneAt = Double.NaN;
        try {
            for (int tick = 0; tick < 35.0 / TICK && Double.isNaN(doneAt); tick++) {
                harness.advance(TICK);
                Thread.sleep(20);
                String step = "";
                for (String line : harness.lastTelemetry()) {
                    if (line.startsWith("step")) {
                        step = line.substring(line.indexOf(':') + 1).trim();
                    }
                }
                timeline.sample(step);
                if (step.equals("done")) {
                    doneAt = timeline.seconds();
                }
            }
        } finally {
            harness.pressStop();
            harness.awaitCompletion(3000);
        }
        System.out.println(auto.getClass().getSimpleName() + "\n  " + timeline.timeline());
        timeline.assertKeptTheRules(auto.getClass().getSimpleName(), doneAt, red);
    }
}
