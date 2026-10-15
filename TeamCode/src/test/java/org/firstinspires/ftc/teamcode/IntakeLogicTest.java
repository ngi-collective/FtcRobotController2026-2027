package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class IntakeLogicTest {

    private static final double DT = 0.02;
    private static final double EPS = 1e-9;

    private static IntakeLogic.Result run(IntakeLogic logic, double trigger, boolean eject, double amps, int loops) {
        IntakeLogic.Result result = null;
        for (int i = 0; i < loops; i++) {
            result = logic.update(trigger, eject, amps, DT);
        }
        return result;
    }

    @Test
    void testIdleWhenNothingPressed() {
        IntakeLogic.Result r = run(new IntakeLogic(), 0.0, false, 0.0, 10);

        assertEquals(0.0, r.power, EPS);
        assertEquals(IntakeLogic.State.IDLE, r.state);
    }

    @Test
    void testFullTriggerRampsUpToIntakePower() {
        IntakeLogic logic = new IntakeLogic();

        IntakeLogic.Result first = logic.update(1.0, false, 0.0, DT);
        assertTrue(first.power > 0);
        assertTrue(first.power <= IntakeLogic.RAMP_PER_SEC * DT + EPS);   // never jumps straight to full

        IntakeLogic.Result settled = run(logic, 1.0, false, 0.0, 100);
        assertEquals(IntakeLogic.INTAKE_POWER, settled.power, EPS);
        assertEquals(IntakeLogic.State.INTAKING, settled.state);
    }

    @Test
    void testHalfTriggerGivesPartialPower() {
        IntakeLogic.Result r = run(new IntakeLogic(), 0.5, false, 0.0, 100);

        assertTrue(r.power > 0.2);
        assertTrue(r.power < IntakeLogic.INTAKE_POWER - 0.1);
    }

    @Test
    void testReleasingTriggerRampsDownToZero() {
        IntakeLogic logic = new IntakeLogic();
        run(logic, 1.0, false, 0.0, 100);

        IntakeLogic.Result r = run(logic, 0.0, false, 0.0, 100);

        assertEquals(0.0, r.power, EPS);
        assertEquals(IntakeLogic.State.IDLE, r.state);
    }

    @Test
    void testEjectWinsOverTriggerAndFlipsSmoothly() {
        IntakeLogic logic = new IntakeLogic();
        run(logic, 1.0, false, 0.0, 100);

        double previous = IntakeLogic.INTAKE_POWER;
        IntakeLogic.Result r = null;
        for (int i = 0; i < 100; i++) {
            r = logic.update(1.0, true, 0.0, DT);   // trigger still held, eject pressed
            assertTrue(Math.abs(r.power - previous) <= IntakeLogic.RAMP_PER_SEC * DT + EPS);
            previous = r.power;
        }

        assertEquals(-IntakeLogic.EJECT_POWER, r.power, EPS);
        assertEquals(IntakeLogic.State.EJECTING, r.state);
    }

    @Test
    void testNormalCurrentNeverTriggersUnjam() {
        IntakeLogic logic = new IntakeLogic();
        double normalAmps = IntakeLogic.STALL_CURRENT_AMPS - 2.0;

        for (int i = 0; i < 300; i++) {
            IntakeLogic.Result r = logic.update(1.0, false, normalAmps, DT);
            assertNotEquals(IntakeLogic.State.UNJAMMING, r.state);
        }
    }

    @Test
    void testStartupInrushIsIgnored() {
        IntakeLogic logic = new IntakeLogic();
        double inrushAmps = IntakeLogic.STALL_CURRENT_AMPS + 1.0;
        int graceLoops = (int) (IntakeLogic.SPINUP_GRACE_SEC / DT) - 2;

        for (int i = 0; i < graceLoops; i++) {
            IntakeLogic.Result r = logic.update(1.0, false, inrushAmps, DT);
            assertNotEquals(IntakeLogic.State.UNJAMMING, r.state);
        }
    }

    @Test
    void testStallReversesThenResumesIntaking() {
        IntakeLogic logic = new IntakeLogic();
        double stallAmps = IntakeLogic.STALL_CURRENT_AMPS + 1.0;

        boolean unjamStarted = false;
        boolean resumed = false;
        double lowestPower = 0.0;

        for (int i = 0; i < 400; i++) {
            double t = (i + 1) * DT;
            // Normal current for the first 0.5 s, then a jam that lasts until the code reverses the roller.
            double amps = (t > 0.5 && !unjamStarted) ? stallAmps : 1.0;
            IntakeLogic.Result r = logic.update(1.0, false, amps, DT);

            if (r.state == IntakeLogic.State.UNJAMMING) unjamStarted = true;
            if (unjamStarted) lowestPower = Math.min(lowestPower, r.power);
            if (unjamStarted && r.state == IntakeLogic.State.INTAKING) resumed = true;
        }

        assertTrue(unjamStarted, "a sustained stall should start an unjam");
        assertTrue(lowestPower < -0.2, "unjam should actually reverse the roller");
        assertTrue(resumed, "intake should resume while the trigger is still held");
    }

    @Test
    void testReleasingTriggerCancelsUnjam() {
        IntakeLogic logic = new IntakeLogic();
        double stallAmps = IntakeLogic.STALL_CURRENT_AMPS + 1.0;
        run(logic, 1.0, false, stallAmps, 80);   // long enough to start an unjam

        IntakeLogic.Result r = run(logic, 0.0, false, stallAmps, 5);

        assertEquals(IntakeLogic.State.IDLE, r.state);
    }
}
