package org.firstinspires.ftc.teamcode.simulated;

import org.firstinspires.ftc.teamcode.pedroPathing.RedRightTipAware;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

/** {@link TipAwareAutoRun} on RedRightTipAware, untipped and tipped. */
@RunWith(RobolectricTestRunner.class)
@Config(sdk = 28)
public class RedRightTipAwareTest {
    @Test
    public void redRightDrivesRoundToTheAudienceSideWhenTheHiveIsUntipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new RedRightTipAware(), TipAwareAutoRun.RED_REAR, "match-setup", true,
                "audience side", TipAwareAutoRun.RED_SHOOT);
    }

    @Test
    public void redRightShootsFromTheRearWhenTheHiveIsTipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new RedRightTipAware(), TipAwareAutoRun.RED_REAR, "tipped-match-setup", true,
                "rear side", TipAwareAutoRun.RED_REAR);
    }
}
