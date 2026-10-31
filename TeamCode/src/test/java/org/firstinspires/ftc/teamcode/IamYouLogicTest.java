package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class IamYouLogicTest {

    private final IamYouLogic logic = new IamYouLogic();

    @Test
    void testSnapToCardinal() {
        assertEquals("NORTH", logic.snapToCardinal(0));
        assertEquals("NORTH", logic.snapToCardinal(44));
        assertEquals("NORTH", logic.snapToCardinal(-44));

        assertEquals("WEST", logic.snapToCardinal(90));
        assertEquals("WEST", logic.snapToCardinal(46));
        assertEquals("WEST", logic.snapToCardinal(134));

        assertEquals("EAST", logic.snapToCardinal(-90));
        assertEquals("EAST", logic.snapToCardinal(-46));
        assertEquals("EAST", logic.snapToCardinal(-134));

        assertEquals("SOUTH", logic.snapToCardinal(180));
        assertEquals("SOUTH", logic.snapToCardinal(-179));
        assertEquals("SOUTH", logic.snapToCardinal(136));
        assertEquals("SOUTH", logic.snapToCardinal(-136));
    }

    @Test
    void testForwardNORTH() {
        // Robot started NORTH, Facing NORTH (heading 0), Driver at SOUTH (baseline)
        // Stick forward (-1.0) -> rawDrive = 1.0
        IamYouLogic.DriveParameters params = new IamYouLogic.DriveParameters(
            -1.0, 0.0, 0.0, 0.0, 0.0, "NORTH", "SOUTH"
        );

        IamYouLogic.DriveResult result = logic.calculate(params);

        // NORTH init: d = drive, s = -strafe
        // d = 1.0, s = 0.0
        // fieldDrive = 1.0, fieldStrafe = 0.0
        // fl = 1.0, fr = 1.0, bl = 1.0, br = 1.0
        assertEquals(1.0, result.powers.getFL(), 1e-6);
        assertEquals(1.0, result.powers.getFR(), 1e-6);
        assertEquals(1.0, result.powers.getBL(), 1e-6);
        assertEquals(1.0, result.powers.getBR(), 1e-6);
    }

    @Test
    void testForwardSOUTH() {
        // Robot started SOUTH, Facing SOUTH (heading 180), Driver at SOUTH
        // Corrected heading = 180 - 180 = 0
        // Stick forward (-1.0) -> rawDrive = 1.0
        IamYouLogic.DriveParameters params = new IamYouLogic.DriveParameters(
            -1.0, 0.0, 0.0, 0.0, 180.0, "SOUTH", "SOUTH"
        );

        IamYouLogic.DriveResult result = logic.calculate(params);

        // SOUTH init: d = -drive, s = strafe
        // d = -1.0, s = 0.0
        // fieldDrive = -1.0, fieldStrafe = 0.0
        // Powers should be -1.0
        assertEquals(-1.0, result.powers.getFL(), 1e-6);
        assertEquals(-1.0, result.powers.getFR(), 1e-6);
        assertEquals(-1.0, result.powers.getBL(), 1e-6);
        assertEquals(-1.0, result.powers.getBR(), 1e-6);
    }

    @Test
    void testSlowMode() {
        // Stick forward, slow mode active
        IamYouLogic.DriveParameters params = new IamYouLogic.DriveParameters(
            -1.0, 0.0, 0.0, 1.0, 0.0, "NORTH", "SOUTH"
        );

        IamYouLogic.DriveResult result = logic.calculate(params);

        assertEquals(IamYouLogic.SLOW_MODE_DRIVE_MULT, result.powers.getFL(), 1e-6);
    }

    @Test
    void testRotationNORTH() {
        // Stick right (-1.0) -> twist = 1.0 (logic negates right stick x)
        IamYouLogic.DriveParameters params = new IamYouLogic.DriveParameters(
            0.0, 0.0, -1.0, 0.0, 0.0, "NORTH", "SOUTH"
        );

        IamYouLogic.DriveResult result = logic.calculate(params);

        // twist = 1.0
        // fl = drive + strafe + twist = 0 + 0 + 1.0 = 1.0
        // bl = 1.0, fr = -1.0, br = -1.0
        assertEquals(1.0, result.powers.getFL(), 1e-6);
        assertEquals(-1.0, result.powers.getFR(), 1e-6);
        assertEquals(1.0, result.powers.getBL(), 1e-6);
        assertEquals(-1.0, result.powers.getBR(), 1e-6);
    }

    @Test
    void testDriverPositionNORTH() {
        // Robot NORTH, Facing NORTH, Driver at NORTH (looking SOUTH)
        // Driver at NORTH -> doff = 180 deg
        // Stick forward (-1.0) -> rawDrive = 1.0
        // Driver rotation: driveR = drive*cos(180) - strafe*sin(180) = -1.0
        // strafeR = 0
        // drive = -1.0
        // initCardinal NORTH: d = drive = -1.0, s = -strafe = 0
        // fieldDrive = -1.0
        
        IamYouLogic.DriveParameters params = new IamYouLogic.DriveParameters(
            -1.0, 0.0, 0.0, 0.0, 0.0, "NORTH", "NORTH"
        );

        IamYouLogic.DriveResult result = logic.calculate(params);

        assertEquals(-1.0, result.powers.getFL(), 1e-6);
    }
}
