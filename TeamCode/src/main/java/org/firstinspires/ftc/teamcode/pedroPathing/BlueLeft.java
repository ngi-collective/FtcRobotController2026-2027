package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Blue, against the rear wall facing the raised CELL: RedLeft turned half a turn. */
@Autonomous(name = "BlueLeft", group = "Autonomous")
public class BlueLeft extends AllianceAuto {
    public BlueLeft() {
        super(BLUE, Start.FACING_RAISED_CELL, Routine.FULL);
    }
}
