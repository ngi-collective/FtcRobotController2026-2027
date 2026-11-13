package org.firstinspires.ftc.teamcode.simulated;

import org.firstinspires.ftc.teamcode.pedroPathing.BlueLeftTipAware;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

/** {@link TipAwareAutoRun} on BlueLeftTipAware, untipped and tipped. */
@RunWith(RobolectricTestRunner.class)
@Config(sdk = 28)
public class BlueLeftTipAwareTest {
    @Test
    public void blueLeftShootsFromTheRearWhenTheHiveIsUntipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new BlueLeftTipAware(), TipAwareAutoRun.BLUE_FRONT, "match-setup", false,
                "rear side", TipAwareAutoRun.BLUE_FRONT);
    }

    @Test
    public void blueLeftDrivesRoundToTheAudienceSideWhenTheHiveIsTipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new BlueLeftTipAware(), TipAwareAutoRun.BLUE_FRONT, "tipped-match-setup", false,
                "audience side", TipAwareAutoRun.BLUE_REAR);
    }
}
