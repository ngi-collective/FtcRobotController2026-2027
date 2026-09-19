package org.firstinspires.ftc.teamcode;

/**
 * What to do with a CELL the camera can see: which way to turn, and how fast to spin the flywheel.
 *
 * <p>The launcher on this robot is fixed at a steep angle and has exactly one control, the wheel's
 * speed. So a shot is fully determined by how far away the target is and how far up: given those,
 * there is one exit speed that drops a ball into the opening, and one wheel speed that produces
 * it. That is the whole of this class, and it is separate from the OpMode because arithmetic that
 * decides whether a ball lands should be testable without a robot, a camera or an emulator.</p>
 *
 * <h2>Aim at the cluster origin</h2>
 *
 * <p>There is no offset table here, and that is a fact about the season rather than a
 * simplification. The SDK reports an {@code AprilTagClusterDetection}'s pose at the cluster's own
 * origin, and on BioBuzz that origin lands <b>within an inch and a half of the centre of the CELL
 * opening</b> the tags are stuck under &mdash; 1.4&nbsp;in below it, an eighth of the opening's
 * own height. Three sources that never mention each other agree there: the SDK's member offsets,
 * FIRST's field CAD, and the manual's 20 &times; 14 &times; 12&nbsp;in CELL. So a detection is
 * already an aim point, and correcting it would mean carrying the CELL's orientation around to
 * apply a correction smaller than the ball. See
 * {@code BioBuzzFieldTest.everyClusterOriginSitsAtItsCellsOpening}.</p>
 *
 * <h2>The shape of a steep lob</h2>
 *
 * <p>At a fixed launch angle {@code p}, a ball that leaves at {@code v} passes a target
 * {@code rise} above the muzzle and {@code d} away when
 * {@code rise = d tan p - g d^2 / (2 v^2 cos^2 p)}, which rearranges to one speed per distance.
 * Two consequences are worth knowing before trusting a number out of here:</p>
 *
 * <ul>
 *   <li>There is a <b>minimum distance</b>, {@code rise / tan p}. Nearer than that the ball cannot
 *       come down steeply enough to be inside the CELL at that height, whatever its speed
 *       &mdash; with this robot's 75&deg; launcher and a raised CELL, about 40&nbsp;cm. A driver
 *       parked under the HIVE cannot score, and the honest answer is to say so rather than to
 *       return a speed that will not work.</li>
 *   <li>The speed needed is <b>not monotonic</b>: it falls to a minimum at {@code 2 rise / tan p}
 *       and climbs on both sides. Closer shots are harder than mid-range ones, which is the
 *       opposite of the intuition a driver brings from a flat shooter.</li>
 * </ul>
 */
public final class ShotSolver {

    /** Standard gravity. The only number here that is not a property of this robot. */
    private static final double GRAVITY_METRES_PER_SECOND_SQUARED = 9.81;

    private final double launchForwardMetres;
    private final double launchLeftMetres;
    private final double launchHeightMetres;
    private final double launchPitchRadians;
    private final double metresPerSecondPerRevolutionPerMinute;
    private final double maxRevolutionsPerMinute;

    private ShotSolver(double launchForwardMetres, double launchLeftMetres,
                       double launchHeightMetres, double launchPitchRadians,
                       double metresPerSecondPerRevolutionPerMinute,
                       double maxRevolutionsPerMinute) {
        this.launchForwardMetres = launchForwardMetres;
        this.launchLeftMetres = launchLeftMetres;
        this.launchHeightMetres = launchHeightMetres;
        this.launchPitchRadians = launchPitchRadians;
        this.metresPerSecondPerRevolutionPerMinute = metresPerSecondPerRevolutionPerMinute;
        this.maxRevolutionsPerMinute = maxRevolutionsPerMinute;
    }

    /**
     * A launcher measured on the robot.
     *
     * @param launchForwardMetres how far ahead of the robot's centre a ball leaves
     * @param launchLeftMetres how far to its left
     * @param launchHeightMetres how far above the tiles &mdash; the centre of the ball as it goes,
     *     not the axle
     * @param launchPitchDegrees how far above level it is thrown
     * @param metresPerSecondPerRevolutionPerMinute the one calibration constant: how much exit
     *     speed a revolution per minute of wheel is worth. Measured, or worked out from the wheel's
     *     radius and how much of its surface speed the ball actually takes away with it.
     * @param maxRevolutionsPerMinute the wheel's free speed, so that a shot nobody can take is
     *     reported as out of range rather than as a command the motor will silently clip
     */
    public static ShotSolver of(double launchForwardMetres, double launchLeftMetres,
                                double launchHeightMetres, double launchPitchDegrees,
                                double metresPerSecondPerRevolutionPerMinute,
                                double maxRevolutionsPerMinute) {
        return new ShotSolver(launchForwardMetres, launchLeftMetres, launchHeightMetres,
                Math.toRadians(launchPitchDegrees), metresPerSecondPerRevolutionPerMinute,
                maxRevolutionsPerMinute);
    }

    /** How fast a ball leaves at a given wheel speed, which is the calibration run forwards. */
    public double exitMetresPerSecond(double revolutionsPerMinute) {
        return revolutionsPerMinute * metresPerSecondPerRevolutionPerMinute;
    }

    /**
     * The shot that puts a ball where the camera is looking.
     *
     * <p>Two steps, in this order because the second depends on the first: turn the robot until the
     * target is on the muzzle's line, then solve the flight along that line. The launcher throws
     * straight out of the nose, so there is nothing to aim but the whole robot.</p>
     */
    public Shot solve(LensMount.Sighting target) {
        return solve(target.forwardMetres(), target.leftMetres(), target.heightMetres());
    }

    /**
     * The same, for a target nothing had to see.
     *
     * <p>An AUTONOMOUS routine starts on a known square of tiles facing a known direction, so
     * where the CELL is relative to the robot comes off the field drawing rather than out of a
     * camera. That is not a lesser way of aiming &mdash; it is the only one available before the
     * robot has moved, and it needs no tag in view &mdash; and it lands in the same arithmetic,
     * which is why this is an overload and not a second solver.</p>
     *
     * @param heightMetres the target's height above the tiles, not above the muzzle
     */
    public Shot solve(double forwardMetres, double leftMetres, double heightMetres) {
        double straightLine = Math.hypot(forwardMetres, leftMetres);

        // Turning the robot swings the muzzle too, so "point at it" is not simply the target's
        // bearing from the centre: what has to become true is that the target is as far to the
        // left as the muzzle is. Solving for that turn rather than for the bearing is what keeps a
        // launcher mounted off the centre line from sitting in a limit cycle, always turned by the
        // offset it is trying to correct.
        double offCentre = launchLeftMetres / straightLine;
        if (!(Math.abs(offCentre) <= 1.0)) {
            // Closer to the robot's centre than its own muzzle is: there is no turn that lines
            // this up, and nothing this near is a CELL.
            return Shot.unreachable(Reach.TOO_CLOSE, 0.0, 0.0, heightMetres);
        }
        double turnRadians = Math.atan2(leftMetres, forwardMetres)
                - Math.asin(offCentre);

        // Once lined up, the flight is along the nose: the distance is what is left of the
        // straight line after the sideways offset and the muzzle's own reach ahead.
        double distanceMetres = Math.sqrt(
                straightLine * straightLine - launchLeftMetres * launchLeftMetres)
                - launchForwardMetres;
        double riseMetres = heightMetres - launchHeightMetres;

        double cosPitch = Math.cos(launchPitchRadians);
        double fall = distanceMetres * Math.tan(launchPitchRadians) - riseMetres;
        if (distanceMetres <= 0.0 || fall <= 0.0) {
            return Shot.unreachable(Reach.TOO_CLOSE, turnRadians, distanceMetres,
                    heightMetres);
        }

        double speedSquared = GRAVITY_METRES_PER_SECOND_SQUARED * distanceMetres * distanceMetres
                / (2.0 * cosPitch * cosPitch * fall);
        double revolutionsPerMinute =
                Math.sqrt(speedSquared) / metresPerSecondPerRevolutionPerMinute;
        if (revolutionsPerMinute > maxRevolutionsPerMinute) {
            return Shot.unreachable(Reach.TOO_FAR, turnRadians, distanceMetres,
                    heightMetres);
        }
        return new Shot(Reach.REACHABLE, turnRadians, distanceMetres, heightMetres,
                revolutionsPerMinute);
    }

    /** Why a shot cannot be taken, when it cannot. */
    public enum Reach {

        /** There is a wheel speed that scores from here. */
        REACHABLE,

        /**
         * Too close for the launch angle: the ball would still be climbing, or not yet falling
         * fast enough, as it passed the CELL's height. Drive back, do not spin faster.
         */
        TOO_CLOSE,

        /** Past what the wheel can throw, even flat out. */
        TOO_FAR,
    }

    /** One solved shot: where to point, and how fast to spin. */
    public static final class Shot {

        private final Reach reach;
        private final double turnRadians;
        private final double distanceMetres;
        private final double targetHeightMetres;
        private final double revolutionsPerMinute;

        private Shot(Reach reach, double turnRadians, double distanceMetres,
                     double targetHeightMetres, double revolutionsPerMinute) {
            this.reach = reach;
            this.turnRadians = turnRadians;
            this.distanceMetres = distanceMetres;
            this.targetHeightMetres = targetHeightMetres;
            this.revolutionsPerMinute = revolutionsPerMinute;
        }

        private static Shot unreachable(Reach reach, double turnRadians, double distanceMetres,
                                        double targetHeightMetres) {
            return new Shot(reach, turnRadians, distanceMetres, targetHeightMetres, Double.NaN);
        }

        /** Whether there is a shot at all, and if not, why not. */
        public Reach reach() {
            return reach;
        }

        public boolean reachable() {
            return reach == Reach.REACHABLE;
        }

        /**
         * How far to turn the robot to line the muzzle up, in radians, counter-clockwise positive.
         *
         * <p>Reported even when the shot is out of range, because a driver rolling backwards out
         * of the minimum distance wants to stay pointed at the CELL while they do it.</p>
         */
        public double turnRadians() {
            return turnRadians;
        }

        /** How far the ball has to fly horizontally, from the muzzle, in metres. */
        public double distanceMetres() {
            return distanceMetres;
        }

        /** How high the target is above the tiles, in metres: which CELL this is, effectively. */
        public double targetHeightMetres() {
            return targetHeightMetres;
        }

        /** The wheel speed to command, or {@code NaN} if this shot cannot be taken. */
        public double revolutionsPerMinute() {
            return revolutionsPerMinute;
        }

        @Override
        public String toString() {
            return String.format("%s: turn %.1f deg, %.2f m to a target %.2f m up, %.0f rpm",
                    reach, Math.toDegrees(turnRadians), distanceMetres, targetHeightMetres,
                    revolutionsPerMinute);
        }
    }
}
