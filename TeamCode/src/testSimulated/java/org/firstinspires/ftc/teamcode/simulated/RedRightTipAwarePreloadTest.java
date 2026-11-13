package org.firstinspires.ftc.teamcode.simulated;

import org.firstinspires.ftc.teamcode.pedroPathing.RedRightTipAwarePreload;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

/** {@link TipAwareAutoRun} on RedRightTipAwarePreload, untipped and tipped. */
@RunWith(RobolectricTestRunner.class)
@Config(sdk = 28)
public class RedRightTipAwarePreloadTest {
    @Test
    public void redRightPreloadWaitsAndShootsFromTheRearWhenTheHiveIsTipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new RedRightTipAwarePreload(), TipAwareAutoRun.RED_REAR, "tipped-match-setup", true,
                "rear side", TipAwareAutoRun.RED_REAR);
    }

    @Test
    public void redRightPreloadDrivesRoundThenWaitsWhenTheHiveIsUntipped() throws Exception {
        TipAwareAutoRun.assertShootsFrom(new RedRightTipAwarePreload(), TipAwareAutoRun.RED_REAR, "match-setup", true,
                "audience side", TipAwareAutoRun.RED_SHOOT);
    }
}
