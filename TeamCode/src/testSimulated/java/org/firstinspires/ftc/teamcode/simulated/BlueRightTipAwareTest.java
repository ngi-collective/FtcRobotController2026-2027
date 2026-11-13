package org.firstinspires.ftc.teamcode.simulated;

import org.firstinspires.ftc.teamcode.pedroPathing.BlueRightTipAware;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

/** {@link TipAwareAutoRun} on BlueRightTipAware, untipped and tipped. */
@RunWith(RobolectricTestRunner.class)
@Config(sdk = 28)
public class BlueRightTipAwareTest {
    @Test
    public void blueRightDrivesRoundToTheRearWhenTheHiveIsUntipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new BlueRightTipAware(), TipAwareAutoRun.BLUE_REAR, "match-setup", false,
                "rear side", TipAwareAutoRun.BLUE_SHOOT);
    }

    @Test
    public void blueRightShootsFromTheAudienceSideWhenTheHiveIsTipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new BlueRightTipAware(), TipAwareAutoRun.BLUE_REAR, "tipped-match-setup", false,
                "audience side", TipAwareAutoRun.BLUE_REAR);
    }
}
