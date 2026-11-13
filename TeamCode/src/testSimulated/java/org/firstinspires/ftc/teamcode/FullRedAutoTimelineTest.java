package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.RedLeft;
import org.firstinspires.ftc.teamcode.pedroPathing.RedRight;
import org.junit.jupiter.api.Test;

/** {@link AllianceAutoRun} on RedLeft, RedRight. */
class FullRedAutoTimelineTest {
    @Test
    void redLeft() throws Exception {
        AllianceAutoRun.run(new RedLeft(), 56, 9.5, 90, true);
    }

    @Test
    void redRight() throws Exception {
        AllianceAutoRun.run(new RedRight(), 62, 134.5, 270, true);
    }
}
