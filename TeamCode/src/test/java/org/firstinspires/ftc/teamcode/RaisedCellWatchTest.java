package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.RaisedCellWatch.Reading;
import org.junit.jupiter.api.Test;

class RaisedCellWatchTest {

    /** Camera frames 33 ms apart, as a 30 fps webcam sends them. */
    private static long frames(RaisedCellWatch watch, long fromMs, long untilMs, boolean cell) {
        long t = fromMs;
        for (; t <= untilMs; t += 33) {
            watch.frame(t, cell);
        }
        return t;
    }

    @Test
    void oneFrameWithTheCellInIsFacingStraightAway() {
        RaisedCellWatch watch = new RaisedCellWatch(0);
        watch.frame(40, true);
        assertEquals(Reading.FACING, watch.reading(40));
        assertTrue(watch.decided(40));
    }

    @Test
    void framesWithoutTheCellAreLookingUntilHalfASecondThenBehind() {
        RaisedCellWatch watch = new RaisedCellWatch(0);
        frames(watch, 100, 400, false);
        assertEquals(Reading.LOOKING, watch.reading(400));
        assertFalse(watch.decided(400));
        frames(watch, 433, 700, false);
        assertEquals(Reading.BEHIND, watch.reading(700));
        assertTrue(watch.decided(700));
    }

    @Test
    void aSightingLapsesAfterHalfASecondOfFramesWithoutIt() {
        RaisedCellWatch watch = new RaisedCellWatch(0);
        watch.frame(0, true);
        frames(watch, 33, 600, false);
        assertEquals(Reading.BEHIND, watch.reading(600));
    }

    @Test
    void noFramesIsNoCameraAndNeverBehind() {
        RaisedCellWatch watch = new RaisedCellWatch(0);
        assertEquals(Reading.NO_CAMERA, watch.reading(5000));
        assertFalse(watch.decided(RaisedCellWatch.LOOK_MS));
        assertTrue(watch.decided(2 * RaisedCellWatch.LOOK_MS));
    }

    @Test
    void aCameraThatStopsSendingIsNoCameraEvenAfterFramesWithoutTheCell() {
        RaisedCellWatch watch = new RaisedCellWatch(0);
        frames(watch, 0, 700, false);
        assertEquals(Reading.NO_CAMERA, watch.reading(1300));
    }

    @Test
    void restartForgetsAnEarlierSighting() {
        RaisedCellWatch watch = new RaisedCellWatch(0);
        watch.frame(10, true);
        watch.restart(20);
        assertEquals(Reading.NO_CAMERA, watch.reading(20));
        assertEquals(0, watch.framesWithCell());
    }
}
