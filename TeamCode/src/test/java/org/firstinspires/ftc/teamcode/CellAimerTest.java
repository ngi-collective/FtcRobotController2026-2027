package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * The turn {@link CellAimer} asks for, away from any camera.
 *
 * <p>Three ways for it to go wrong and all three look like a bad aimer rather than a bad number:
 * a sign that turns away from the CELL, a missing floor that stalls a few degrees short, and a
 * missing ceiling that spins past.</p>
 */
class CellAimerTest {

    private static final double EPSILON = 1e-9;

    @Test
    void aCellToTheLeftTurnsCounterClockwise() {
        assertEquals(0.2 * CellAimer.TURN_GAIN_PER_RADIAN, CellAimer.turnPower(0.2), EPSILON);
        assertEquals(-0.2 * CellAimer.TURN_GAIN_PER_RADIAN, CellAimer.turnPower(-0.2), EPSILON);
    }

    @Test
    void aSmallErrorStillTurnsHardEnoughToMove() {
        assertEquals(CellAimer.TURN_FLOOR, CellAimer.turnPower(Math.toRadians(3.0)), EPSILON);
        assertEquals(-CellAimer.TURN_FLOOR, CellAimer.turnPower(Math.toRadians(-3.0)), EPSILON);
    }

    @Test
    void aLargeErrorTurnsNoFasterThanTheCeiling() {
        assertEquals(CellAimer.TURN_CEILING, CellAimer.turnPower(Math.toRadians(120.0)), EPSILON);
        assertEquals(-CellAimer.TURN_CEILING, CellAimer.turnPower(Math.toRadians(-120.0)),
                EPSILON);
    }

    @Test
    void theErrorIsTheCellsBearingFromTheRobotsCentre() {
        // The launcher sits on the centre line and the robot turns about its centre, so pointing
        // the launcher is pointing the centre: no allowance for the muzzle's reach ahead.
        assertEquals(0.0, CellAimer.errorRadians(0.5, 0.0), EPSILON);
        assertEquals(Math.toRadians(45.0), CellAimer.errorRadians(0.4, 0.4), EPSILON);
        assertEquals(Math.toRadians(-45.0), CellAimer.errorRadians(0.4, -0.4), EPSILON);
    }
}
