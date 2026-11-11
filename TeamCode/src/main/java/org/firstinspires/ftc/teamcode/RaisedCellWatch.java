package org.firstinspires.ftc.teamcode;

/**
 * Whether the alliance's raised CELL faces the robot, worked out from camera frames over time.
 *
 * <p>All four tags of a HIVE face the side its raised CELL opens toward. A camera on that side sees
 * the raised cluster; a camera on the other side sees nothing of that HIVE at all, because from
 * there the plates are edge-on or turned away. So "facing" is something the camera can see, and
 * "behind" can only be inferred: frames keep arriving and none of them has the cluster in. A
 * camera that has stopped sending frames proves nothing, and is reported as such rather than
 * read as a TIP. See {@code docs/adr/0007-the-camera-looks-forward-and-up.md}.</p>
 *
 * <p>No SDK types, so it can be tested on a plain JVM: the OpMode tells it when a frame arrives
 * and whether {@link CellAimer#raisedCell} found anything in it.</p>
 */
public final class RaisedCellWatch {

    /** What the camera says about the alliance's raised CELL. */
    public enum Reading {
        /** The raised cluster was seen in the last {@link #LOOK_MS}: the raised CELL faces us. */
        FACING,
        /** Frames for at least {@link #LOOK_MS} and none had the cluster: it faces the other way. */
        BEHIND,
        /** Frames are arriving, but not yet for long enough to call it {@link #BEHIND}. */
        LOOKING,
        /** No frame in the last {@link #LOOK_MS}: the camera is missing, opening or broken. */
        NO_CAMERA
    }

    /**
     * How long, in milliseconds, a sighting counts, and how long the camera must look without one
     * before the answer is {@link Reading#BEHIND}.
     *
     * <p>Fifteen frames at the webcam's 30 a second: enough that one frame the detector missed
     * does not turn a CELL round, and short enough to spend before every shot.</p>
     */
    public static final long LOOK_MS = 500;

    private long lookingSinceMs;
    private long firstFrameMs = -1;
    private long lastFrameMs = -1;
    private long lastSightingMs = -1;
    private int framesWithCell;

    public RaisedCellWatch(long nowMs) {
        restart(nowMs);
    }

    /** Forgets every frame so far: what follows is a fresh look from wherever the robot is now. */
    public void restart(long nowMs) {
        lookingSinceMs = nowMs;
        firstFrameMs = -1;
        lastFrameMs = -1;
        lastSightingMs = -1;
        framesWithCell = 0;
    }

    /**
     * One camera frame.
     *
     * @param sawRaisedCell whether the frame had this alliance's raised cluster in it
     */
    public void frame(long nowMs, boolean sawRaisedCell) {
        if (firstFrameMs < 0) {
            firstFrameMs = nowMs;
        }
        lastFrameMs = nowMs;
        if (sawRaisedCell) {
            lastSightingMs = nowMs;
            framesWithCell++;
        }
    }

    public Reading reading(long nowMs) {
        if (lastSightingMs >= 0 && nowMs - lastSightingMs <= LOOK_MS) {
            return Reading.FACING;
        }
        if (lastFrameMs < 0 || nowMs - lastFrameMs > LOOK_MS) {
            return Reading.NO_CAMERA;
        }
        if (nowMs - firstFrameMs >= LOOK_MS) {
            return Reading.BEHIND;
        }
        return Reading.LOOKING;
    }

    /**
     * Whether a look started at {@link #restart} has an answer worth acting on.
     *
     * <p>{@link Reading#FACING} or {@link Reading#BEHIND} as soon as either is true, and
     * {@link Reading#NO_CAMERA} once the camera has had twice {@link #LOOK_MS} to send anything.</p>
     */
    public boolean decided(long nowMs) {
        Reading reading = reading(nowMs);
        return reading == Reading.FACING || reading == Reading.BEHIND
                || (reading == Reading.NO_CAMERA && nowMs - lookingSinceMs >= 2 * LOOK_MS);
    }

    /** How many frames since {@link #restart} had the raised cluster in, for telemetry. */
    public int framesWithCell() {
        return framesWithCell;
    }
}
