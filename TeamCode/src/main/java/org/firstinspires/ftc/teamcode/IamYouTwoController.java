package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * I AM YOU for two drivers: gamepad 1 drives, gamepad 2 runs the intake and outtake.
 * Everything else, controls included, is in {@link iamyou}.
 */
@TeleOp(name = "I AM YOU (2 controllers)")
public class IamYouTwoController extends iamyou {

    @Override
    protected boolean twoControllers() {
        return true;
    }
}
