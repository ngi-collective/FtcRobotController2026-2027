package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * The pickup loop collecting blue nectar. Mirrors {@link RedPickupAuto} for the other alliance.
 */
@Autonomous(name = "Blue Pickup", group = "Pickup")
public class BluePickupAuto extends PickupAuto {

    /**
     * The blue loading zone, mirrored across the field's centre line from red. Placeholder: replace
     * with the measured start square the blue alliance actually uses.
     */
    private static final Pose BLUE_LOADING_ZONE = new Pose(
            -0.55 * INCHES_PER_METRE, 1.43 * INCHES_PER_METRE, -Math.PI / 2.0);

    @Override
    protected PickupLogic.OwnColor ownColor() {
        return PickupLogic.OwnColor.BLUE;
    }

    @Override
    protected Pose startingPose() {
        return BLUE_LOADING_ZONE;
    }

    @Override
    protected Pose parkingPose() {
        return BLUE_LOADING_ZONE;
    }
}
