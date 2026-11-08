package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Red, against the rear wall facing the lowered CELL: opposite RedLeft. */
@Autonomous(name = "RedRight", group = "Autonomous")
public class RedRight extends AllianceAuto {
    public RedRight() {
        super(RED, Start.BEHIND_LOWERED_CELL);
    }
}
