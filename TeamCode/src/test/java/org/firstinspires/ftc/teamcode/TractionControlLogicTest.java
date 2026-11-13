package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.Test;

import static org.firstinspires.ftc.teamcode.TractionControlLogic.GripState;
import static org.firstinspires.ftc.teamcode.TractionControlLogic.Odometry;
import static org.firstinspires.ftc.teamcode.TractionControlLogic.PinpointHealth;
import static org.firstinspires.ftc.teamcode.TractionControlLogic.Result;
import static org.firstinspires.ftc.teamcode.TractionControlLogic.WheelVelocities;
import static org.junit.jupiter.api.Assertions.*;

class TractionControlLogicTest {

    private static final double DT = 0.02;
    private static final double EPS = 1e-9;

    private static final ChassisMotors FULL_FORWARD = new ChassisMotors(1, 1, 1, 1);

    /** Encoder ticks per second for a wheel surface speed. */
    private static double ticks(double metresPerSecond) {
        return metresPerSecond / (2 * Math.PI * TractionConstants.WHEEL_RADIUS_METRES * TractionConstants.GEAR_RATIO)
                * TractionConstants.TICKS_PER_REV;
    }

    private static WheelVelocities wheels(double fl, double fr, double bl, double br) {
        return new WheelVelocities(ticks(fl), ticks(fr), ticks(bl), ticks(br));
    }

    private static WheelVelocities allWheels(double metresPerSecond) {
        return wheels(metresPerSecond, metresPerSecond, metresPerSecond, metresPerSecond);
    }

    private static Odometry ready(double velX, double velY, double heading, double headingRate) {
        return new Odometry(PinpointHealth.READY, velX, velY, heading, headingRate);
    }

    private static Result step(TractionControlLogic logic, ChassisMotors commanded, WheelVelocities measured,
                               Odometry odometry, boolean bypass) {
        logic.observe(odometry);
        return logic.update(commanded, measured, bypass, DT);
    }

    private static void assertNoSlip(Result r) {
        assertEquals(0.0, r.flSlip, 1e-6);
        assertEquals(0.0, r.frSlip, 1e-6);
        assertEquals(0.0, r.blSlip, 1e-6);
        assertEquals(0.0, r.brSlip, 1e-6);
        assertEquals(1.0, r.scale, EPS);
        assertEquals(GripState.FULL_GRIP, r.state);
    }

    // ------------------------------------------------------------------ what is not slip

    @Test
    void wheelsMatchingTheGroundAreFullGrip() {
        TractionControlLogic logic = new TractionControlLogic();
        Result r = null;
        for (int i = 0; i < 50; i++) {
            r = step(logic, FULL_FORWARD, allWheels(1.2), ready(1.2, 0, 0, 0), false);
        }
        assertNoSlip(r);
        assertEquals(1.0, r.powers.getFL(), EPS);
    }

    @Test
    void theStrafeLossIsNotReadAsSlip() {
        // Strafing left: the chassis slides at STRAFE_EFFICIENCY of the wheels' surface speed.
        double wheel = 1.0;
        double chassisLeft = wheel * TractionConstants.STRAFE_EFFICIENCY;
        TractionControlLogic logic = new TractionControlLogic();
        Result r = step(logic, new ChassisMotors(-1, 1, 1, -1),
                wheels(-wheel, wheel, wheel, -wheel), ready(0, chassisLeft, 0, 0), false);
        assertNoSlip(r);
    }

    @Test
    void pinpointVelocityIsRotatedIntoTheRobotFrame() {
        // Facing field +Y (heading 90°) and driving straight ahead: the Pinpoint reports field velocity along Y.
        // Read unrotated, that is a strafe, and every wheel would look badly slipping.
        TractionControlLogic logic = new TractionControlLogic();
        Result r = step(logic, FULL_FORWARD, allWheels(1.0), ready(0, 1.0, Math.PI / 2, 0), false);
        assertNoSlip(r);
    }

    @Test
    void turningOnTheSpotIsNotSlip() {
        double yawRate = 2.0; // counter-clockwise
        double surface = yawRate * (TractionConstants.TRACK_WIDTH_METRES + TractionConstants.WHEEL_BASE_METRES) / 2;
        TractionControlLogic logic = new TractionControlLogic();
        Result r = step(logic, new ChassisMotors(-1, 1, -1, 1),
                wheels(-surface, surface, -surface, surface), ready(0, 0, 0.3, yawRate), false);
        assertNoSlip(r);
    }

    // ------------------------------------------------------------------ spin and lockup

    @Test
    void standingStartSpinCutsAllFourWheelsEquallyDownToTheFloor() {
        TractionControlLogic logic = new TractionControlLogic();
        Result first = step(logic, new ChassisMotors(1, 0.5, 0.5, 1), allWheels(1.0), ready(0, 0, 0, 0), false);

        // 1 m/s of wheel over a stationary chassis, measured against the low-speed floor.
        assertEquals(1.0 / TractionConstants.SLIP_SPEED_FLOOR_METRES_PER_SECOND, first.flSlip, 1e-6);
        assertEquals(GripState.HEAVY_SLIP, first.state);
        assertTrue(first.scale < 1.0);
        // One factor on every wheel, so the commanded direction survives.
        assertEquals(first.scale, first.powers.getFL(), EPS);
        assertEquals(first.scale * 0.5, first.powers.getFR(), EPS);
        assertEquals(first.scale * 0.5, first.powers.getBL(), EPS);
        assertEquals(first.scale, first.powers.getBR(), EPS);

        Result r = first;
        for (int i = 0; i < 200; i++) {
            r = step(logic, FULL_FORWARD, allWheels(1.0), ready(0, 0, 0, 0), false);
        }
        assertEquals(TractionConstants.MIN_SCALE, r.scale, EPS);
    }

    @Test
    void brakingLockupCutsPowerToo() {
        // Moving forward at 1 m/s while the driver slams into reverse and the wheels already turn backwards.
        TractionControlLogic logic = new TractionControlLogic();
        Result r = step(logic, new ChassisMotors(-1, -1, -1, -1), allWheels(-0.3), ready(1.0, 0, 0, 0), false);
        assertTrue(r.flSlip < -TractionConstants.TARGET_SLIP);
        assertTrue(r.scale < 1.0);
    }

    @Test
    void slipUnderTargetLeavesPowerAlone() {
        double chassis = 1.0;
        double wheel = chassis * (1 + TractionConstants.TARGET_SLIP * 0.9);
        TractionControlLogic logic = new TractionControlLogic();
        Result r = null;
        for (int i = 0; i < 50; i++) {
            r = step(logic, FULL_FORWARD, allWheels(wheel), ready(chassis, 0, 0, 0), false);
        }
        assertEquals(1.0, r.scale, EPS);
    }

    @Test
    void recoveryIsRateLimited() {
        TractionControlLogic logic = new TractionControlLogic();
        for (int i = 0; i < 200; i++) {
            step(logic, FULL_FORWARD, allWheels(1.0), ready(0, 0, 0, 0), false);
        }
        // Grip is back: the scale climbs at RECOVERY_PER_SECOND, not instantly.
        Result r = null;
        for (int i = 0; i < 5; i++) {
            r = step(logic, FULL_FORWARD, allWheels(1.0), ready(1.0, 0, 0, 0), false);
        }
        assertEquals(TractionConstants.MIN_SCALE + 5 * DT * TractionConstants.RECOVERY_PER_SECOND, r.scale, 1e-9);
        assertEquals(GripState.LIMITED, r.state);
    }

    @Test
    void slipOnUnpoweredWheelsCannotCutPower() {
        // Stick released while coasting on braked wheels: scaling a zero command changes nothing, so the scale
        // must not sit low and make the next push sluggish.
        TractionControlLogic logic = new TractionControlLogic();
        Result r = null;
        for (int i = 0; i < 50; i++) {
            r = step(logic, new ChassisMotors(0, 0, 0, 0), allWheels(0.0), ready(1.0, 0, 0, 0), false);
        }
        assertEquals(1.0, r.scale, EPS);
        assertEquals(0.0, r.worstPoweredSlip, EPS);
        assertEquals(-1.0, r.flSlip, 1e-6, "slip is still reported");
    }

    // ------------------------------------------------------------------ bypass

    @Test
    void bypassPassesPowersThroughButStillReportsSlip() {
        TractionControlLogic logic = new TractionControlLogic();
        for (int i = 0; i < 50; i++) {
            step(logic, FULL_FORWARD, allWheels(1.0), ready(0, 0, 0, 0), false);
        }
        Result r = step(logic, FULL_FORWARD, allWheels(1.0), ready(0, 0, 0, 0), true);
        assertEquals(1.0, r.scale, EPS);
        assertEquals(1.0, r.powers.getFL(), EPS);
        assertEquals(GripState.HEAVY_SLIP, r.state);
    }

    // ------------------------------------------------------------------ Pinpoint faults

    @Test
    void oneBadReadIsCoveredByTheLastGoodReading() {
        TractionControlLogic logic = new TractionControlLogic();
        step(logic, FULL_FORWARD, allWheels(1.0), ready(0, 1.0, Math.PI / 2, 0), false);

        Odometry garbage = new Odometry(PinpointHealth.BAD_READ, 1e6, -1e6, 3.0, 50.0);
        assertTrue(logic.observe(garbage));
        assertEquals(Math.PI / 2, logic.headingRadians(), EPS);
        assertNoSlip(logic.update(FULL_FORWARD, allWheels(1.0), false, DT));

        assertFalse(logic.observe(garbage), "a second bad read in a row fails open");
        assertTrue(Double.isNaN(logic.headingRadians()));
    }

    @Test
    void untrustedOdometryFailsOpenAndRestartsAtFullScale() {
        TractionControlLogic logic = new TractionControlLogic();
        for (int i = 0; i < 200; i++) {
            step(logic, FULL_FORWARD, allWheels(1.0), ready(0, 0, 0, 0), false);
        }

        Result fault = step(logic, new ChassisMotors(0.7, 0.7, 0.7, 0.7), allWheels(1.0),
                new Odometry(PinpointHealth.FAULT, 0, 0, 0, 0), false);
        assertFalse(fault.odometryTrusted);
        assertEquals(GripState.ODOMETRY_FAULT, fault.state);
        assertEquals(0.7, fault.powers.getFL(), EPS);
        assertTrue(Double.isNaN(fault.flSlip));

        // A bad read straight after a fault has no good reading to fall back on.
        assertFalse(logic.observe(new Odometry(PinpointHealth.BAD_READ, 0, 0, 0, 0)));

        Result back = step(logic, FULL_FORWARD, allWheels(1.0), ready(1.0, 0, 0, 0), false);
        assertEquals(1.0, back.scale, EPS);
    }

    @Test
    void badReadBeforeAnyGoodReadingIsUntrusted() {
        assertFalse(new TractionControlLogic().observe(new Odometry(PinpointHealth.BAD_READ, 0, 0, 0, 0)));
    }
}
