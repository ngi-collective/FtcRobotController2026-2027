package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.ngicollective.testframework.camera.Pose3d;
import org.ngicollective.testframework.camera.TagCluster;
import org.ngicollective.testframework.camera.Vec3;
import org.ngicollective.testframework.season.BioBuzzField;
import org.ngicollective.testframework.season.BioBuzzHive;
import org.ngicollective.testframework.sim.LauncherConfig;
import org.ngicollective.testframework.sim.RobotConfig;
import org.ngicollective.testframework.sim.SimConfigFiles;

/**
 * That the numbers an OpMode aims with are the numbers the robot is built from.
 *
 * <p>{@link LaunchGeometry} is a second copy of part of {@code verity.json}, and it has to be: the
 * competition APK ships no configuration files and a Control Hub has no checkout, so robot code
 * carries its own measurements. The copy is safe only while something notices when it stops
 * matching, because nothing else does. Re-aim the camera in the configuration and every shot
 * misses, with the solved range, the commanded speed and the achieved speed all still agreeing
 * with each other on the Driver Station.</p>
 */
class LaunchGeometryTest {

    private static final double INCH = 0.0254;

    private static RobotConfig verity() {
        return SimConfigFiles.robot("verity");
    }

    @Test
    void theOpModesLensMountIsTheConfiguredOne() {
        // Through the pose the renderer itself looks along, not through the six numbers: the
        // simulator turns a mount into a pose in exactly one place, so comparing against that
        // place is what makes this an agreement rather than a second opinion.
        Pose3d simulated = verity().camera().mount().pose();

        // Sighted rather than compared field by field, because what matters is that the two agree
        // about where a thing the camera sees actually is, and a mount is six numbers that only
        // mean something together. One oblique sight line exercises all six at once -- a mount
        // wrong in yaw and right in pitch still fails this.
        Vec3 expected = simulated.position()
                .plus(simulated.forward().scaled(1.3))
                .minus(simulated.left().scaled(0.4))
                .plus(simulated.up().scaled(0.25));

        LensMount.Sighting sighted = LaunchGeometry.camera().sight(0.4, 1.3, 0.25);

        assertEquals(expected.x(), sighted.forwardMetres(), 1e-9,
                "LaunchGeometry's camera is not where verity.json bolts it");
        assertEquals(expected.y(), sighted.leftMetres(), 1e-9);
        assertEquals(expected.z(), sighted.heightMetres(), 1e-9);
    }

    @Test
    void theOpModesLauncherThrowsAsFastAsTheSimulatedOneDoes() {
        LauncherConfig launcher = verity().launchers().get("flywheel");
        double freeRevolutionsPerMinute = verity().motors().get("flywheel").rpm();

        // The calibration, from the other side: the configuration says a wheel of this radius
        // turning this fast gives a ball this much of its surface speed, and the OpMode says a
        // revolution per minute is worth so many metres per second. At free speed they have to
        // land on the same exit speed, or the solver is solving for a launcher this robot has not
        // got.
        double simulated = 2.0 * Math.PI * launcher.wheelRadiusMetres() * freeRevolutionsPerMinute
                / 60.0 * launcher.transferEfficiency();

        assertEquals(simulated, LaunchGeometry.solver().exitMetresPerSecond(
                freeRevolutionsPerMinute), 1e-9);
    }

    @Test
    void theOpModeThrowsFromWhereTheMouthIs() {
        LauncherConfig launcher = verity().launchers().get("flywheel");

        // A ball leaves from inside the mouth, so the muzzle the solver measures from has to be
        // within it. Asserting the mouth's own span rather than its centre, because the mouth is
        // 6 cm long and pinning the centre would break every time it was widened.
        //
        // Sighted through a mount at the robot's own origin, so that the target's distance ahead
        // is exactly the two metres asked for and what is left over is the muzzle's reach.
        double targetForwardMetres = 2.0;
        ShotSolver.Shot shot = LaunchGeometry.solver().solve(
                LensMount.of(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                        .sight(0.0, targetForwardMetres, 1.5100));
        double muzzleForward = targetForwardMetres - shot.distanceMetres();

        double nearEdge = launcher.mouth().forwardMetres() - launcher.mouth().lengthMetres() / 2.0;
        double farEdge = launcher.mouth().forwardMetres() + launcher.mouth().lengthMetres() / 2.0;
        assertTrue(muzzleForward >= nearEdge - 1e-9 && muzzleForward <= farEdge + 1e-9,
                "the solver throws from " + muzzleForward + " m ahead, and the mouth a ball is"
                        + " actually in spans " + nearEdge + " to " + farEdge + " m");
    }

    /**
     * And the one number neither file holds: how high a raised CELL's opening is.
     *
     * <p>The gate between a CELL worth shooting at and its partner facing the floor. Both carry a
     * tag cluster on the same side of the field, so the only thing separating them is height, and
     * a gate on the wrong side of either would have the robot aiming at the one that scores
     * nothing &mdash; with a perfectly plausible range and speed.</p>
     */
    @Test
    void theHeightGateFallsBetweenTheTwoCellsOfAHive() {
        double raised = openingHeight(BioBuzzField.RED_AUDIENCE, BioBuzzField.HiveTip.AUDIENCE_UP);
        double lowered = openingHeight(BioBuzzField.RED_SCORING, BioBuzzField.HiveTip.AUDIENCE_UP);

        assertEquals(59.45, raised / INCH, 0.5, "the manual's raised CELL opening");
        assertTrue(lowered < LaunchGeometry.RAISED_CELL_HEIGHT_METRES,
                "the CELL facing the floor is at " + lowered + " m, which the gate at "
                        + LaunchGeometry.RAISED_CELL_HEIGHT_METRES + " m must exclude");
        assertTrue(raised > LaunchGeometry.RAISED_CELL_HEIGHT_METRES,
                "and the one worth shooting at is at " + raised + " m, which it must not");
    }

    /**
     * And that a cluster detection can be aimed at directly, which is what
     * {@link ShotSolver} assumes.
     *
     * <p>Checked here as well as in {@code BioBuzzFieldTest} because it is the assumption an
     * OpMode makes rather than a fact about the field: if the SDK's cluster origin were anywhere
     * else, this robot would need a per-CELL offset table and there is nowhere to put one.</p>
     */
    @Test
    void whatTheCameraReportsIsAlreadyTheAimPoint() {
        for (TagCluster cluster : BioBuzzField.clusters(
                BioBuzzField.HiveTip.AUDIENCE_UP, BioBuzzField.HiveTip.AUDIENCE_UP)) {
            Pose3d cell = BioBuzzHive.cell(cluster.name(), BioBuzzField.HiveTip.AUDIENCE_UP,
                    BioBuzzField.HiveTip.AUDIENCE_UP);
            Vec3 opening = cell.position()
                    .plus(cell.forward().scaled(BioBuzzHive.CELL_DEPTH_METRES / 2.0));

            double missMetres = cluster.pose().position().minus(opening).length();
            assertTrue(missMetres < BioBuzzHive.CELL_RISE_METRES / 4.0,
                    cluster.name() + ": aiming at the reported cluster origin misses the middle"
                            + " of the opening by " + missMetres + " m, which is more than a"
                            + " quarter of the opening's own height");
        }
    }

    private static double openingHeight(String cell, BioBuzzField.HiveTip tip) {
        Pose3d pose = BioBuzzHive.cell(cell, tip, tip);
        return pose.position().z()
                + pose.forward().scaled(BioBuzzHive.CELL_DEPTH_METRES / 2.0).z();
    }
}
