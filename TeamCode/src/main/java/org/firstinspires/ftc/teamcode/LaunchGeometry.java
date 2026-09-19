package org.firstinspires.ftc.teamcode;

/**
 * Everything about this robot that a shot depends on, measured once and written down here.
 *
 * <p>One place, because every one of these numbers appears in two calculations at least, and a
 * launcher that is 2&nbsp;cm lower than the arithmetic thinks misses low at every range without a
 * single line of telemetry looking wrong. The simulator holds the same numbers in
 * {@code TeamCode/robot-config/verity.json} and places its camera and its flywheel from them;
 * robot code cannot read that file, because the competition APK does not ship it and a Control Hub
 * has no checkout. So this is deliberately a second copy, and
 * {@code LaunchGeometryTest} is what keeps the two honest.</p>
 *
 * <h2>Which CELL is worth shooting at</h2>
 *
 * <p>A HIVE has two CELLs and only the raised one scores (manual &sect;10.5.1), and both of them
 * carry a tag cluster on the same side of the field &mdash; the two plates share a normal, so a
 * camera sees all four of a HIVE's tags at once and gets two aim points back. Height is what
 * separates them: a raised CELL's opening is 1.51&nbsp;m above the tiles and a lowered one's is
 * 0.97&nbsp;m, which is not a close call. {@link #RAISED_CELL_HEIGHT_METRES} sits between them.</p>
 */
public final class LaunchGeometry {

    /** Measured on the robot: the lens is on the front face of the chassis, aimed up at the HIVE. */
    private static final double CAMERA_FORWARD_METRES = 0.16;
    private static final double CAMERA_LEFT_METRES = 0.0;
    private static final double CAMERA_HEIGHT_METRES = 0.105;
    private static final double CAMERA_YAW_DEGREES = 0.0;
    private static final double CAMERA_PITCH_DEGREES = 35.0;
    private static final double CAMERA_ROLL_DEGREES = 0.0;

    /**
     * Where a ball is when it leaves: in the mouth, ahead of the nose, sitting on the tiles.
     *
     * <p>The height is the centre of a POLLEN resting on the floor &mdash; 1.4&nbsp;in, half of
     * its 2.8&nbsp;in diameter &mdash; and not the axle of anything, because what flies is the
     * ball's centre of mass and the drop from the shooter's own geometry is already spent by the
     * time it is moving.</p>
     */
    private static final double LAUNCH_FORWARD_METRES = 0.24;
    private static final double LAUNCH_LEFT_METRES = 0.0;
    private static final double LAUNCH_HEIGHT_METRES = 0.0356;

    /**
     * How steeply it throws, in degrees above level.
     *
     * <p>Steep by necessity. The CELL's opening is tilted 30&deg; and faces up and inward, so a
     * flat shot arrives at the lip rather than through the hole, and the manual's own 18&nbsp;in
     * robot height limit puts the muzzle four feet below the target.</p>
     */
    private static final double LAUNCH_PITCH_DEGREES = 75.0;

    /** The flywheel: a 4 in wheel on a bare 6000 rpm motor. */
    private static final double WHEEL_RADIUS_METRES = 0.0508;
    private static final double FLYWHEEL_FREE_REVOLUTIONS_PER_MINUTE = 6000.0;

    /**
     * What fraction of the wheel's surface speed a ball leaves with.
     *
     * <p>A ball squeezed between a wheel and a hood leaves at roughly half the surface speed: it
     * is also spinning, and the grip is not perfect. Half is the number to measure first and
     * trust least &mdash; it is the single constant that scales every range, so a shooter that is
     * consistently 10&nbsp;% short is telling you this figure, not the arithmetic.</p>
     */
    private static final double TRANSFER_EFFICIENCY = 0.5;

    /**
     * Exit speed per revolution per minute: the whole calibration, in one number.
     *
     * <p>Derived from the wheel rather than written down, so that changing the wheel changes the
     * shots. A revolution per minute is {@code 2 pi r / 60} of surface speed, of which the ball
     * takes {@link #TRANSFER_EFFICIENCY}.</p>
     */
    private static final double METRES_PER_SECOND_PER_REVOLUTION_PER_MINUTE =
            2.0 * Math.PI * WHEEL_RADIUS_METRES * TRANSFER_EFFICIENCY / 60.0;

    /**
     * Above this height, in metres, a CELL's opening is the raised one.
     *
     * <p>Halfway between the two, near enough: a raised opening is at 1.51&nbsp;m and a lowered
     * one at 0.97&nbsp;m, so nothing about where this sits between them is delicate. What it is
     * really rejecting is a detection of the CELL on the floor side of a HIVE, which is a perfectly
     * good aim point at a target that scores nothing.</p>
     */
    public static final double RAISED_CELL_HEIGHT_METRES = 1.25;

    private LaunchGeometry() {
    }

    /** Where the camera is bolted, for turning what it sees into where that is. */
    public static LensMount camera() {
        return LensMount.of(CAMERA_FORWARD_METRES, CAMERA_LEFT_METRES, CAMERA_HEIGHT_METRES,
                CAMERA_YAW_DEGREES, CAMERA_PITCH_DEGREES, CAMERA_ROLL_DEGREES);
    }

    /** The launcher, ready to be asked what a given target needs. */
    public static ShotSolver solver() {
        return ShotSolver.of(LAUNCH_FORWARD_METRES, LAUNCH_LEFT_METRES, LAUNCH_HEIGHT_METRES,
                LAUNCH_PITCH_DEGREES, METRES_PER_SECOND_PER_REVOLUTION_PER_MINUTE,
                FLYWHEEL_FREE_REVOLUTIONS_PER_MINUTE);
    }

    /**
     * How far ahead of the robot's centre a ball leaves, in metres.
     *
     * <p>Needed by an AUTONOMOUS routine, which measures its target from a row of balls on the
     * tiles rather than from the chassis: a ball flies from where it is lying, and the solver
     * takes this off again.</p>
     */
    public static double muzzleForwardMetres() {
        return LAUNCH_FORWARD_METRES;
    }
}
