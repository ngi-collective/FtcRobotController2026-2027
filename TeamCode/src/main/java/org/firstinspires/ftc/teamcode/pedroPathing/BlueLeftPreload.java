package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BlueLeft's start: shoots the pre-loads at once, then parks. */
@Autonomous(name = "BlueLeft Preload", group = "Autonomous")
public class BlueLeftPreload extends AllianceAuto {
    public BlueLeftPreload() {
        super(BLUE, Start.FACING_RAISED_CELL, Routine.PRELOAD_AND_PARK);
    }
}
