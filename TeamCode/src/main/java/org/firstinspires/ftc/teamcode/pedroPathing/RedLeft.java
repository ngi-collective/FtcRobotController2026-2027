package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Red, against the audience wall facing the raised CELL. */
@Autonomous(name = "RedLeft", group = "Autonomous")
public class RedLeft extends AllianceAuto {
    public RedLeft() {
        super(RED, Start.FACING_RAISED_CELL);
    }
}
