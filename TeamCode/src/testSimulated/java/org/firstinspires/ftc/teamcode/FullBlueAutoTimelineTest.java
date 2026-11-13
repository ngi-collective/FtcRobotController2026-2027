package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.BlueLeft;
import org.firstinspires.ftc.teamcode.pedroPathing.BlueRight;
import org.junit.jupiter.api.Test;

/** {@link AllianceAutoRun} on BlueLeft, BlueRight. */
class FullBlueAutoTimelineTest {
    @Test
    void blueLeft() throws Exception {
        AllianceAutoRun.run(new BlueLeft(), 88, 134.5, 270, false);
    }

    @Test
    void blueRight() throws Exception {
        AllianceAutoRun.run(new BlueRight(), 82, 9.5, 90, false);
    }
}
