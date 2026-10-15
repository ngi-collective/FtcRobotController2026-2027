package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

/**
 * The pickup's target policy, exercised over every combination of colour and range.
 *
 * <p>Own nectar beats everything, then pollen, and the opponent's nectar is never a target &mdash;
 * the decoy rule that makes the two OpModes behave differently is exactly what this pins down.</p>
 */
class PickupLogicTest {

    private static BallDetector.Detection detection(BallDetector.Kind kind, double range) {
        return new BallDetector.Detection(kind, 0.0, 0.0, 0.0, range, 0.0, 0.0);
    }

    @Test
    void ownNectarIsChosenBeforePollen() {
        BallDetector.Detection nectar = detection(BallDetector.Kind.RED_NECTAR, 1.5);
        BallDetector.Detection pollen = detection(BallDetector.Kind.POLLEN, 0.5);

        assertEquals(nectar, PickupLogic.selectTarget(
                Arrays.asList(pollen, nectar), PickupLogic.OwnColor.RED));
    }

    @Test
    void pollenIsChosenWhenNoOwnNectarIsVisible() {
        BallDetector.Detection pollen = detection(BallDetector.Kind.POLLEN, 0.7);

        assertEquals(pollen, PickupLogic.selectTarget(
                Collections.singletonList(pollen), PickupLogic.OwnColor.RED));
    }

    @Test
    void theOpponentsNectarIsNeverATarget() {
        BallDetector.Detection decoy = detection(BallDetector.Kind.BLUE_NECTAR, 0.2);
        BallDetector.Detection pollen = detection(BallDetector.Kind.POLLEN, 1.0);

        // A red robot ignores the blue decoy and takes the pollen instead.
        assertEquals(pollen, PickupLogic.selectTarget(
                Arrays.asList(decoy, pollen), PickupLogic.OwnColor.RED));
        assertNull(PickupLogic.selectTarget(
                Collections.singletonList(decoy), PickupLogic.OwnColor.RED));
    }

    @Test
    void theNearestOwnNectarIsChosen() {
        BallDetector.Detection near = detection(BallDetector.Kind.BLUE_NECTAR, 0.4);
        BallDetector.Detection far = detection(BallDetector.Kind.BLUE_NECTAR, 1.2);

        assertEquals(near, PickupLogic.selectTarget(
                Arrays.asList(far, near), PickupLogic.OwnColor.BLUE));
    }
}
