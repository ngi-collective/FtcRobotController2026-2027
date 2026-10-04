package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * That a solved shot actually lands in the CELL.
 *
 * <p>Checked by flying the ball rather than by rearranging the same equation, which would only
 * assert that algebra is reversible. {@link #flightHeightAt} integrates the motion in small steps
 * from the muzzle and reports how high the ball is when it gets there, so a sign error, a missing
 * {@code cos}, or the wrong root of the quadratic all show up as a ball at the wrong height.</p>
 */
class ShotSolverTest {

    private static final double GRAVITY = 9.81;

    /** A launcher on the nose, throwing steeply, with a plausible flywheel. */
    private static final double MUZZLE_FORWARD = 0.24;
    private static final double MUZZLE_HEIGHT = 0.04;
    private static final double PITCH_DEGREES = 75.0;
    private static final double METRES_PER_SECOND_PER_RPM = 0.00266;
    private static final double FREE_RPM = 6000.0;

    private static final double CELL_HEIGHT = 1.51;

    private static ShotSolver solver() {
        return ShotSolver.of(MUZZLE_FORWARD, 0.0, MUZZLE_HEIGHT, PITCH_DEGREES,
                METRES_PER_SECOND_PER_RPM, FREE_RPM);
    }

    /** A target dead ahead of the robot, that far from its centre and that high. */
    private static LensMount.Sighting aheadAt(double forwardMetres, double heightMetres) {
        return LensMount.of(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                .sight(0.0, forwardMetres, heightMetres);
    }

    @Test
    void aSolvedShotFallsIntoTheCellAtEveryRangeThatHasOne() {
        // Five stand-offs spanning the useful band: hard up against the minimum, at the easiest
        // distance, and out to the far side of the field. One range would not notice a solution
        // that is right at its own calibration point and wrong on the slope either side.
        for (double centreToTarget : new double[] {0.7, 1.0, 1.3, 2.0, 3.2}) {
            ShotSolver.Shot shot = solver().solve(aheadAt(centreToTarget, CELL_HEIGHT));
            assertTrue(shot.reachable(),
                    centreToTarget + " m out should be a shot, got " + shot);

            double landed = flightHeightAt(shot.distanceMetres(),
                    shot.revolutionsPerMinute() * METRES_PER_SECOND_PER_RPM);
            assertEquals(CELL_HEIGHT, landed, 0.01,
                    "from " + centreToTarget + " m the ball passed the CELL at " + landed
                            + " m instead of " + CELL_HEIGHT + ": " + shot);
        }
    }

    @Test
    void theRobotsOwnLauncherWantsTheSpeedTheSimulatorWasTunedTo() {
        // Two entirely separate routes to one number. 2100 rev/min is the figure LauncherTeleOp
        // carries as its starting point, arrived at by watching balls land in the simulator; this
        // is the arithmetic from the measured geometry, which never saw a ball. They agree to a
        // couple of rev/min, so either the physics and the ballistics are both right or they are
        // wrong in the same way, and the second is not a mistake anyone makes twice independently.
        ShotSolver.Shot shot = LaunchGeometry.solver()
                .solve(aheadAt(0.24 + 0.86, 1.5100));

        assertEquals(0.86, shot.distanceMetres(), 1e-9);
        assertEquals(2100.0, shot.revolutionsPerMinute(), 5.0, shot.toString());
    }

    @Test
    void underTheHiveThereIsNoShotAtAnySpeed() {
        // A steep lob still cannot fall vertically. Inside rise/tan(pitch) the ball is at the
        // CELL's height while still on its way up, or not yet dropping fast enough to be inside
        // the opening, and no wheel speed changes that: more speed only sends it higher and
        // further. A solver that answered with a number here would have a driver parked under the
        // HIVE spinning the wheel up and wondering.
        double minimum = (CELL_HEIGHT - MUZZLE_HEIGHT) / Math.tan(Math.toRadians(PITCH_DEGREES));
        assertEquals(0.394, minimum, 0.002, "the geometry this test is about");

        ShotSolver.Shot tooClose =
                solver().solve(aheadAt(MUZZLE_FORWARD + minimum - 0.05, CELL_HEIGHT));
        assertFalse(tooClose.reachable());
        assertEquals(ShotSolver.Reach.TOO_CLOSE, tooClose.reach());

        // And just past it there is one, which is what makes the boundary a boundary rather than
        // a blanket refusal.
        assertTrue(solver().solve(aheadAt(MUZZLE_FORWARD + minimum + 0.15, CELL_HEIGHT))
                .reachable());
    }

    @Test
    void closeShotsAreHarderThanMidRangeOnes() {
        // The surprise in a steep launcher, and the reason a driver cannot just "get closer for an
        // easier shot": the speed needed bottoms out at twice the minimum distance and climbs on
        // both sides. A sign slip in the fall term leaves the solution monotonic, which reads
        // perfectly sensibly and is wrong everywhere.
        double rise = CELL_HEIGHT - MUZZLE_HEIGHT;
        double easiest = 2.0 * rise / Math.tan(Math.toRadians(PITCH_DEGREES));

        double atEasiest = rpmAtDistance(easiest);
        double nearer = rpmAtDistance(easiest - 0.25);
        double further = rpmAtDistance(easiest + 0.25);

        assertTrue(nearer > atEasiest,
                "a quarter metre closer should need more, not less: " + nearer + " vs " + atEasiest);
        assertTrue(further > atEasiest,
                "and so should a quarter metre further: " + further + " vs " + atEasiest);
    }

    @Test
    void acrossTheFieldIsOutOfRangeRatherThanACommandTheMotorWillClip() {
        // The whole field is 3.58 m, so this is a target no BioBuzz robot can be this far from;
        // the point is that the answer is "out of range" and not a speed the controller silently
        // saturates, which would look like a shot that is merely mis-tuned.
        ShotSolver.Shot shot = solver().solve(aheadAt(40.0, CELL_HEIGHT));

        assertFalse(shot.reachable());
        assertEquals(ShotSolver.Reach.TOO_FAR, shot.reach());
        assertTrue(Double.isNaN(shot.revolutionsPerMinute()),
                "an unreachable shot must not hand back a speed to command");
    }

    @Test
    void theTurnLinesUpTheMuzzleRatherThanTheRobotsCentre() {
        // A launcher 8 cm to the left of centre. Turning by the target's own bearing would leave
        // the muzzle pointing 8 cm past it, and since turning swings the muzzle too, correcting
        // again never converges. The fixed point is the target sitting as far to the left as the
        // muzzle is, so that is what the solver reports a turn to.
        double muzzleLeft = 0.08;
        ShotSolver offset = ShotSolver.of(MUZZLE_FORWARD, muzzleLeft, MUZZLE_HEIGHT, PITCH_DEGREES,
                METRES_PER_SECOND_PER_RPM, FREE_RPM);

        LensMount.Sighting seen = LensMount.of(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                .sight(0.35, 1.2, CELL_HEIGHT);
        ShotSolver.Shot shot = offset.solve(seen);

        // Turn the robot by what it said, which rotates the target the other way in the robot's
        // frame, and the target should now be exactly abeam the muzzle.
        double turn = shot.turnRadians();
        double forward = seen.forwardMetres() * Math.cos(turn) + seen.leftMetres() * Math.sin(turn);
        double left = -seen.forwardMetres() * Math.sin(turn) + seen.leftMetres() * Math.cos(turn);

        assertEquals(muzzleLeft, left, 1e-9,
                "after turning " + Math.toDegrees(turn) + " deg the target should be on the"
                        + " muzzle's line, not the chassis centre's");
        assertEquals(forward - MUZZLE_FORWARD, shot.distanceMetres(), 1e-9,
                "and the range should be measured from the muzzle along that line");
    }

    private static double rpmAtDistance(double muzzleToTarget) {
        ShotSolver.Shot shot =
                solver().solve(aheadAt(MUZZLE_FORWARD + muzzleToTarget, CELL_HEIGHT));
        assertTrue(shot.reachable(), muzzleToTarget + " m should be reachable");
        return shot.revolutionsPerMinute();
    }

    /**
     * How high the ball is, having flown that far horizontally, integrated in 0.1&nbsp;ms steps.
     *
     * <p>Deliberately not the closed form the solver uses. Explicit Euler over a flight this short
     * is good to well under the centimetre the assertions allow, and it knows nothing about how
     * the speed was chosen.</p>
     */
    private static double flightHeightAt(double horizontalMetres, double exitMetresPerSecond) {
        double pitch = Math.toRadians(PITCH_DEGREES);
        double velocityForward = exitMetresPerSecond * Math.cos(pitch);
        double velocityUp = exitMetresPerSecond * Math.sin(pitch);

        double step = 1e-4;
        double travelled = 0.0;
        double height = MUZZLE_HEIGHT;
        while (travelled < horizontalMetres) {
            travelled += velocityForward * step;
            height += velocityUp * step;
            velocityUp -= GRAVITY * step;
        }
        return height;
    }
}
