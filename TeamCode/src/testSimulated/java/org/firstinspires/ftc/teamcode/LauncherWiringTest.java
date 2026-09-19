package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.simulated.VerityRobot;
import org.junit.jupiter.api.Test;
import org.ngicollective.testframework.hardware.FakeHardwareMap;
import org.ngicollective.testframework.sim.LauncherConfig;
import org.ngicollective.testframework.sim.RobotConfig;
import org.ngicollective.testframework.sim.SimConfigFiles;

/**
 * That the robot's launcher, as configured, behaves like something with a flywheel on it.
 *
 * <p>The physics of throwing a ball is tested against a robot the test writes, in
 * {@code LauncherTest}. What cannot be tested there is the wiring: that the numbers in
 * {@code verity.json} reach the simulated devices, and in particular that
 * {@code spinUpSeconds} becomes a motor with inertia. Every other motor on this robot is ideal and
 * reaches its commanded speed between two ticks; a launcher that did the same would make firing
 * too early &mdash; the mistake the whole mechanism exists to let an OpMode make &mdash; impossible
 * to reproduce.</p>
 */
class LauncherWiringTest {

    private static final double TICK = 0.02;

    @Test
    void theConfiguredFlywheelTakesItsSpinUpTimeToReachSpeed() {
        RobotConfig config = SimConfigFiles.robot("verity");
        LauncherConfig launcher = config.launchers().values().iterator().next();
        assertNotNull(launcher, "verity.json should declare a launcher");

        FakeHardwareMap hardware = new VerityRobot().create();
        double fullSpeed = hardware.motor(launcher.motorName()).state().getMaxTicksPerSecond();
        hardware.motor(launcher.motorName()).setPower(1.0);

        // One tick in: a long way short, because a flywheel has mass. This is the assertion that
        // fails if the launcher's motor is built with the default ideal behaviour.
        hardware.advance(TICK);
        double afterOneTick = hardware.motor(launcher.motorName()).getVelocity();
        assertTrue(afterOneTick < fullSpeed / 10.0,
                "a flywheel commanded for 20 ms should be nowhere near " + fullSpeed
                        + " ticks/s, but it was already at " + afterOneTick);

        // And by its spin-up time it is there, which is the other half of the contract: an OpMode
        // that waits is allowed to expect the speed it asked for.
        for (double waited = TICK; waited < launcher.spinUpSeconds(); waited += TICK) {
            hardware.advance(TICK);
        }
        assertEquals(fullSpeed, hardware.motor(launcher.motorName()).getVelocity(), fullSpeed * 0.02,
                "after " + launcher.spinUpSeconds() + " s the flywheel should be up to speed");
    }

    /**
     * And that the mouth is somewhere a ball can actually be.
     *
     * <p>A box tucked behind the bumper would be a mouth nothing could ever be in, because a ball
     * inside the footprint is evicted from under the robot. That is invisible in a configuration
     * file &mdash; it is a relation between two blocks of it &mdash; and it reads as a launcher
     * that simply never fires.</p>
     */
    @Test
    void theConfiguredMouthReachesPastTheBumperAndDownToTheFloor() {
        RobotConfig config = SimConfigFiles.robot("verity");
        LauncherConfig launcher = config.launchers().values().iterator().next();

        double nose = config.chassis().lengthMetres() / 2.0;
        double nearEdge = launcher.mouth().forwardMetres() - launcher.mouth().lengthMetres() / 2.0;
        assertTrue(nearEdge >= nose - 1e-9,
                "the mouth starts " + nearEdge + " m out, which is inside a nose at " + nose);

        double floorEdge = launcher.mouth().heightMetres() - launcher.mouth().tallMetres() / 2.0;
        assertTrue(floorEdge <= 0.0 + 1e-9,
                "the mouth's floor is " + floorEdge + " m up, so a ball lying on the tiles is"
                        + " below it and this launcher can only fire what something lifts into it");
    }
}
