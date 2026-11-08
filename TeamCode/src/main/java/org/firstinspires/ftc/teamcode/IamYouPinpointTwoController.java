package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * I AM YOU with field-centric heading from the goBILDA Pinpoint, two controllers: gamepad 1 drives, gamepad 2 runs
 * the intake and flywheel. Everything else, controls included, is in {@link iamyou}.
 */
@TeleOp(name = "I AM YOU Pinpoint (2 controllers)")
public class IamYouPinpointTwoController extends iamyou {

    @Override
    protected boolean twoControllers() {
        return true;
    }

    @Override
    protected boolean usePinpoint() {
        return true;
    }
}
