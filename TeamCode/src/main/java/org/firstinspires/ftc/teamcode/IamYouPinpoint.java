package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * I AM YOU with field-centric heading from the goBILDA Pinpoint, one controller.
 * Everything else, controls included, is in {@link iamyou}.
 */
@TeleOp(name = "I AM YOU Pinpoint (1 controller)")
public class IamYouPinpoint extends iamyou {

    @Override
    protected boolean usePinpoint() {
        return true;
    }
}
