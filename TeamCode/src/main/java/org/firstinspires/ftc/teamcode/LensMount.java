package org.firstinspires.ftc.teamcode;

/**
 * Where a camera is bolted to this robot, and what that does to everything it reports.
 *
 * <p>A {@code VisionPortal} hands back positions in the <em>camera's</em> frame: {@code x} to the
 * right of the image, {@code y} out along the lens, {@code z} up the image. That frame is only the
 * robot's frame if the camera is pointed straight ahead and sitting level, and on this robot it is
 * neither &mdash; it aims up at the CELLs, because BioBuzz puts its AprilTags on the undersides of
 * things three to four feet in the air.</p>
 *
 * <h2>Why {@code ftcPose.range} is not the range you want</h2>
 *
 * <p>The SDK's convenient fields are all measured in the camera's own frame:
 * {@code range = hypot(x, y)} leaves out {@code z} entirely, so it is a horizontal distance
 * <em>in the plane of the image</em>, and {@code bearing} and {@code elevation} are angles about
 * the camera's own tilted axes. Aim a camera up by 35&deg; and every one of them is wrong about
 * the field by an amount that looks plausible: at a metre's stand-off, a target three feet up
 * reads roughly 20&nbsp;% short. Nothing in the numbers says so, and a launcher tuned against
 * them is tuned against the tilt.</p>
 *
 * <p>So the only fields this class consumes are {@code x}, {@code y} and {@code z}, which are the
 * raw translation and carry no convention beyond their axes, and the levelling is done here where
 * the mount is known. What comes out is measured against the tiles, which is the frame gravity
 * works in and therefore the only frame a ballistic solution can be written in.</p>
 *
 * <h2>Why the robot cannot read this out of its configuration</h2>
 *
 * <p>{@code TeamCode/robot-config/verity.json} holds these six numbers as well, and the simulator
 * places the camera from them. This class cannot read that file: the competition APK does not ship
 * it, and a real Control Hub has no checkout to read. A robot's OpMode has to carry its own
 * measurements &mdash; so these are a second copy, and
 * {@code LaunchGeometryTest} is what stops the two from drifting apart. That test
 * exists because the drift is silent: every shot simply misses.</p>
 */
public final class LensMount {

    /** Where the lens sits, in the robot's frame, in metres. */
    private final double forwardMetres;
    private final double leftMetres;
    private final double heightMetres;

    /**
     * The camera's own axes in the robot's frame.
     *
     * <p>Held as three unit vectors rather than as the angles they came from, because that is what
     * every sighting needs and it is nine multiplications either way. The components are
     * {@code [forward, left, up]}, which is this project's robot frame: right-handed with
     * {@code +X} out of the nose and {@code +Z} up.</p>
     */
    private final double[] lensAxis;
    private final double[] leftAxis;
    private final double[] upAxis;

    private LensMount(double forwardMetres, double leftMetres, double heightMetres,
                        double yawRadians, double pitchRadians, double rollRadians) {
        this.forwardMetres = forwardMetres;
        this.leftMetres = leftMetres;
        this.heightMetres = heightMetres;

        double cosYaw = Math.cos(yawRadians);
        double sinYaw = Math.sin(yawRadians);
        double cosPitch = Math.cos(pitchRadians);
        double sinPitch = Math.sin(pitchRadians);
        double cosRoll = Math.cos(rollRadians);
        double sinRoll = Math.sin(rollRadians);

        // The three columns of Rz(yaw) * Ry(-pitch) * Rx(roll), which is the same chart the
        // simulator's Pose3d uses. Copied deliberately rather than approximated: yaw and pitch
        // alone would be enough for the mount this robot actually has, and a camera later shimmed
        // a few degrees off level would then be wrong in a way no telemetry line shows.
        this.lensAxis = new double[] {cosPitch * cosYaw, cosPitch * sinYaw, sinPitch};
        this.leftAxis = new double[] {
                -cosYaw * sinPitch * sinRoll - sinYaw * cosRoll,
                -sinYaw * sinPitch * sinRoll + cosYaw * cosRoll,
                cosPitch * sinRoll};
        this.upAxis = new double[] {
                -cosYaw * sinPitch * cosRoll + sinYaw * sinRoll,
                -sinYaw * sinPitch * cosRoll - cosYaw * sinRoll,
                cosPitch * cosRoll};
    }

    /**
     * A mount measured with a ruler and a protractor.
     *
     * @param forwardMetres how far ahead of the robot's centre the lens is
     * @param leftMetres how far to its left
     * @param heightMetres how far above the tiles
     * @param yawDegrees which way it looks, counter-clockwise from the nose
     * @param pitchDegrees how far up from level, positive upward
     * @param rollDegrees how far the image is turned about the lens
     */
    public static LensMount of(double forwardMetres, double leftMetres, double heightMetres,
                                 double yawDegrees, double pitchDegrees, double rollDegrees) {
        return new LensMount(forwardMetres, leftMetres, heightMetres,
                Math.toRadians(yawDegrees), Math.toRadians(pitchDegrees),
                Math.toRadians(rollDegrees));
    }

    /**
     * Where something the camera reported actually is, measured against the robot and the tiles.
     *
     * <p>The three arguments are {@code AprilTagDetection.ftcPose}'s {@code x}, {@code y} and
     * {@code z} in that order, in metres &mdash; build the processor with
     * {@code setOutputUnits(DistanceUnit.METER, ...)} and they arrive in metres.</p>
     */
    public Sighting sight(double rightMetres, double alongLensMetres, double upMetres) {
        double forward = forwardMetres
                + lensAxis[0] * alongLensMetres
                - leftAxis[0] * rightMetres
                + upAxis[0] * upMetres;
        double left = leftMetres
                + lensAxis[1] * alongLensMetres
                - leftAxis[1] * rightMetres
                + upAxis[1] * upMetres;
        double height = heightMetres
                + lensAxis[2] * alongLensMetres
                - leftAxis[2] * rightMetres
                + upAxis[2] * upMetres;
        return new Sighting(forward, left, height);
    }

    /** How high the lens itself is, which is what makes a sighting's height absolute. */
    public double heightMetres() {
        return heightMetres;
    }

    /**
     * Something the camera saw, placed in the robot's frame with its height above the tiles.
     *
     * <p>Height rather than "up from the camera", because every question worth asking of it is
     * about the field: is that CELL the raised one, how far does a ball have to climb. A
     * camera-relative rise would need the mount again at every use.</p>
     */
    public static final class Sighting {

        private final double forwardMetres;
        private final double leftMetres;
        private final double heightMetres;

        private Sighting(double forwardMetres, double leftMetres, double heightMetres) {
            this.forwardMetres = forwardMetres;
            this.leftMetres = leftMetres;
            this.heightMetres = heightMetres;
        }

        /** How far ahead of the robot's centre, in metres. */
        public double forwardMetres() {
            return forwardMetres;
        }

        /** How far to the robot's left, in metres; negative is to its right. */
        public double leftMetres() {
            return leftMetres;
        }

        /** How far above the tiles, in metres. */
        public double heightMetres() {
            return heightMetres;
        }

        @Override
        public String toString() {
            return String.format("%.3f m ahead, %.3f m left, %.3f m up",
                    forwardMetres, leftMetres, heightMetres);
        }
    }
}
