package org.firstinspires.ftc.teamcode;

import java.util.List;

/**
 * The decisions an autonomous pickup makes between seeing and driving.
 *
 * <p>Pure Java, because every decision here is ordinary arithmetic over a list of detections: which
 * ball to chase next is a policy, and a policy that nothing exercises on a plain JVM is a policy
 * that rots. The OpMode feeds it detections and drives wherever it points; the policy's only job is
 * the pointing.</p>
 */
public final class PickupLogic {

    /** Which colour of nectar this robot is collecting; the other colour is the opponent's decoy. */
    public enum OwnColor {
        RED, BLUE
    }

    private PickupLogic() {
    }

    /**
     * The ball to chase next, or null when the frame holds nothing worth chasing.
     *
     * <p>Own-colour nectar first &mdash; the nearest &mdash; and pollen only when no own nectar is
     * in view, which keeps the robot productive instead of idling. The opponent's nectar is never a
     * target: it is a decoy placed to waste a robot that cannot tell the two colours apart.</p>
     */
    public static BallDetector.Detection selectTarget(List<BallDetector.Detection> detections,
                                                      OwnColor own) {
        BallDetector.Detection bestNectar = null;
        BallDetector.Detection bestPollen = null;
        for (BallDetector.Detection detection : detections) {
            if (detection.kind() == BallDetector.Kind.POLLEN) {
                if (bestPollen == null
                        || detection.rangeMetres() < bestPollen.rangeMetres()) {
                    bestPollen = detection;
                }
            } else if (isOwnNectar(detection.kind(), own)) {
                if (bestNectar == null
                        || detection.rangeMetres() < bestNectar.rangeMetres()) {
                    bestNectar = detection;
                }
            }
            // The opponent's nectar falls through: never a target.
        }
        return bestNectar != null ? bestNectar : bestPollen;
    }

    /** Whether a nectar detection is this robot's own colour rather than the opponent's decoy. */
    private static boolean isOwnNectar(BallDetector.Kind kind, OwnColor own) {
        if (own == OwnColor.RED) {
            return kind == BallDetector.Kind.RED_NECTAR;
        }
        return kind == BallDetector.Kind.BLUE_NECTAR;
    }
}
