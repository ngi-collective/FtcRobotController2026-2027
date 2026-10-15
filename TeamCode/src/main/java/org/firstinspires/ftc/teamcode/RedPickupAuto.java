package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * The pickup loop collecting red nectar. Mirrors {@link BluePickupAuto} for the other alliance.
 */
@Autonomous(name = "Red Pickup", group = "Pickup")
public class RedPickupAuto extends PickupAuto {

    /**
     * The red loading zone, where the robot starts and parks. FTC field frame &mdash; metres,
     * origin at field centre, heading 0 facing +X &mdash; converted to PedroPathing's inches.
     *
     * <p>Placeholder: replace with the measured start square the red alliance actually uses.</p>
     */
    private static final Pose RED_LOADING_ZONE = new Pose(
            -0.55 * INCHES_PER_METRE, -1.43 * INCHES_PER_METRE, Math.PI / 2.0);

    @Override
    protected PickupLogic.OwnColor ownColor() {
        return PickupLogic.OwnColor.RED;
    }

    @Override
    protected Pose startingPose() {
        return RED_LOADING_ZONE;
    }

    @Override
    protected Pose parkingPose() {
        return RED_LOADING_ZONE;
    }
}
