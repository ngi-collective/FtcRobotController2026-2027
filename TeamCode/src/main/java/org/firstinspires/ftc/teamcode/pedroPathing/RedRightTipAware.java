package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** RedRight's start and full routine, shooting at whichever CELL the camera says is raised. */
@Autonomous(name = "RedRight Tip-Aware", group = "Autonomous")
public class RedRightTipAware extends TipAwareAuto {
    public RedRightTipAware() {
        super(AllianceAuto.RED, Start.REAR);
    }
}
