package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * That a camera aimed up does not report distances as though it were level.
 *
 * <p>Every assertion here is a number a tape measure could check, because the whole class exists
 * to undo one rotation and the way that goes wrong is a sign or an axis, not a decimal place.</p>
 */
class LensMountTest {

    /** Verity's own mount: on the nose, a hand's width up, aimed 35 degrees above level. */
    private static LensMount verity() {
        return LensMount.of(0.16, 0.0, 0.105, 0.0, 35.0, 0.0);
    }

    @Test
    void aTagCentredInATiltedImageIsNotStraightAhead() {
        // A metre along the lens of a camera pitched up 35 degrees. The SDK would call this
        // "range 1.0", because its range is measured in the image's own plane -- and a launcher
        // told the target is a metre away when it is 82 cm away and 57 cm up is a launcher aiming
        // at the wrong solution entirely.
        LensMount.Sighting seen = verity().sight(0.0, 1.0, 0.0);

        assertEquals(0.16 + Math.cos(Math.toRadians(35.0)), seen.forwardMetres(), 1e-9);
        assertEquals(0.0, seen.leftMetres(), 1e-9);
        assertEquals(0.105 + Math.sin(Math.toRadians(35.0)), seen.heightMetres(), 1e-9);

        // Spelled out, since this is the mistake the class prevents: a fifth of the distance and
        // most of the height, both silently.
        assertEquals(0.979, seen.forwardMetres(), 0.001);
        assertEquals(0.679, seen.heightMetres(), 0.001);
    }

    @Test
    void theRaisedCellComesOutWhereTheManualPutsIt() {
        // The season's own numbers, end to end. A robot a metre back from the HIVE, tags 1.475 m
        // above the lens along a sight line up and forwards: the answer has to be a target at the
        // height the manual gives for a raised CELL's opening, 1.51 m, or the height gate that
        // tells a raised CELL from a lowered one is gating on fiction.
        double alongLens = Math.hypot(1.10 - 0.16, 1.5100 - 0.105);
        double pitchToTarget = Math.atan2(1.5100 - 0.105, 1.10 - 0.16);
        double offAxis = pitchToTarget - Math.toRadians(35.0);

        LensMount.Sighting seen = verity().sight(
                0.0, alongLens * Math.cos(offAxis), alongLens * Math.sin(offAxis));

        assertEquals(1.10, seen.forwardMetres(), 1e-6);
        assertEquals(1.5100, seen.heightMetres(), 1e-6);
    }

    @Test
    void rightInTheImageIsRightOnTheRobot() {
        // The one axis the SDK and this project disagree about: images measure x to the right and
        // robots measure y to the left. Get it backwards and the assist turns away from the CELL,
        // which is unmistakable in a log and completely invisible in the arithmetic.
        LensMount.Sighting seen = LensMount.of(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                .sight(0.5, 2.0, 0.0);

        assertEquals(-0.5, seen.leftMetres(), 1e-9, "a tag to the right is at negative left");
        assertEquals(2.0, seen.forwardMetres(), 1e-9);
    }

    @Test
    void aYawedCameraLooksWhereItIsPointed() {
        // A camera turned 90 degrees to the left: what it calls "along the lens" is the robot's
        // left. This is the mount a side-looking camera has, and it shares every line of the
        // arithmetic with the pitch case.
        LensMount.Sighting seen = LensMount.of(0.1, 0.0, 0.2, 90.0, 0.0, 0.0)
                .sight(0.0, 1.5, 0.0);

        assertEquals(0.1, seen.forwardMetres(), 1e-9);
        assertEquals(1.5, seen.leftMetres(), 1e-9);
        assertEquals(0.2, seen.heightMetres(), 1e-9);
    }

    @Test
    void rollTurnsTheImageAndLeavesTheLensAlone() {
        // A camera mounted on its side still looks the same way; only up and across swap over.
        // Worth pinning because roll is the term a mount description omits, and a mount that
        // silently ignored it would agree with this robot's own zero and be wrong for the next
        // one built.
        LensMount.Sighting alongLens = LensMount.of(0.0, 0.0, 0.0, 0.0, 0.0, 90.0)
                .sight(0.0, 1.0, 0.0);
        assertEquals(1.0, alongLens.forwardMetres(), 1e-9,
                "roll is about the lens, so it cannot move the lens");

        LensMount.Sighting upTheImage = LensMount.of(0.0, 0.0, 0.0, 0.0, 0.0, 90.0)
                .sight(0.0, 0.0, 1.0);
        assertEquals(0.0, upTheImage.heightMetres(), 1e-9);
        assertTrue(Math.abs(upTheImage.leftMetres()) > 0.99,
                "rolled a quarter turn, the top of the image is out to one side, not up:"
                        + " got " + upTheImage);
    }
}
