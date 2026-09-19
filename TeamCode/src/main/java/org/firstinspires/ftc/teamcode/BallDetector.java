package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds BioBuzz POLLEN and NECTAR in an RGBA camera frame and ranges each by apparent size.
 *
 * <p>Pure Java on purpose: the whole pipeline &mdash; HSV threshold, connected components,
 * classification, and the size-based range &mdash; runs on a plain JVM in milliseconds, which is
 * what makes the detection-tolerance test possible. On the robot, an EasyOpenCV processor hands
 * over the same RGBA bytes and reuses this class unchanged.</p>
 *
 * <p>The three balls are unambiguous by colour and size: POLLEN is yellow and 7.1 cm, NECTAR is
 * red or blue and 9.1 cm. Colour classifies; the classified radius is what makes range fall out of
 * apparent size, because a ball of radius {@code r} at depth {@code d} subtends {@code f * r / d}
 * pixels through a pinhole lens.</p>
 */
public final class BallDetector {

    /** BioBuzz POLLEN, 2.8 in across. */
    public static final double POLLEN_RADIUS_METRES = 0.071 / 2.0;

    /** BioBuzz NECTAR, 3.6 in across, red or blue. */
    public static final double NECTAR_RADIUS_METRES = 0.091 / 2.0;

    /** Below this saturation a pixel is a floor, a wall or a tag, not a ball. */
    private static final double SATURATION_MIN = 0.45;

    /** Below this value a pixel is too dark to trust its hue. */
    private static final double VALUE_MIN = 0.35;

    /** A blob with fewer pixels than this is noise, not a ball. */
    private static final int MIN_BLOB_PIXELS = 3;

    private static final byte NONE = 0;
    private static final byte YELLOW = 1;
    private static final byte RED = 2;
    private static final byte BLUE = 3;

    private BallDetector() {
    }

    /** The kind of ball a blob's colour implies. */
    public enum Kind {
        POLLEN, RED_NECTAR, BLUE_NECTAR
    }

    /** One classified ball, in the camera's frame. */
    public static final class Detection {
        private final Kind kind;
        private final double pixelX;
        private final double pixelY;
        private final double apparentRadiusPixels;
        private final double rangeMetres;
        private final double bearingRadians;
        private final double elevationRadians;

        Detection(Kind kind, double pixelX, double pixelY, double apparentRadiusPixels,
                  double rangeMetres, double bearingRadians, double elevationRadians) {
            this.kind = kind;
            this.pixelX = pixelX;
            this.pixelY = pixelY;
            this.apparentRadiusPixels = apparentRadiusPixels;
            this.rangeMetres = rangeMetres;
            this.bearingRadians = bearingRadians;
            this.elevationRadians = elevationRadians;
        }

        public Kind kind() {
            return kind;
        }

        /** Centroid x, pixels from the left edge. */
        public double pixelX() {
            return pixelX;
        }

        /** Centroid y, pixels from the top edge. */
        public double pixelY() {
            return pixelY;
        }

        /** The blob's radius in pixels, {@code sqrt(area / pi)}. */
        public double apparentRadiusPixels() {
            return apparentRadiusPixels;
        }

        /** Depth along the optical axis, from {@code f * r / radiusPixels}. */
        public double rangeMetres() {
            return rangeMetres;
        }

        /** Horizontal angle from the optical axis, right-positive. */
        public double bearingRadians() {
            return bearingRadians;
        }

        /** Vertical angle from the optical axis, down-positive. */
        public double elevationRadians() {
            return elevationRadians;
        }
    }

    /**
     * Detects every ball in one RGBA frame.
     *
     * @param rgba    row-major RGBA bytes, four per pixel, as {@code Mat.put} expects
     * @param width   frame width in pixels
     * @param height  frame height in pixels
     * @param focalX  horizontal focal length in pixels
     * @param focalY  vertical focal length in pixels
     * @param centreX principal point x in pixels
     * @param centreY principal point y in pixels
     */
    public static List<Detection> detect(byte[] rgba, int width, int height,
                                         double focalX, double focalY,
                                         double centreX, double centreY) {
        byte[] label = new byte[width * height];
        for (int i = 0, p = 0; p < rgba.length; i++, p += 4) {
            label[i] = classify(rgba[p] & 0xFF, rgba[p + 1] & 0xFF, rgba[p + 2] & 0xFF);
        }

        List<Detection> found = new ArrayList<>();
        boolean[] visited = new boolean[width * height];
        int[] queue = new int[width * height];
        for (int start = 0; start < label.length; start++) {
            if (label[start] == NONE || visited[start]) {
                continue;
            }
            byte blobLabel = label[start];
            int head = 0;
            int tail = 0;
            queue[tail++] = start;
            visited[start] = true;
            long sumX = 0;
            long sumY = 0;
            long area = 0;
            while (head < tail) {
                int p = queue[head++];
                int x = p % width;
                int y = p / width;
                sumX += x;
                sumY += y;
                area++;
                if (x > 0 && !visited[p - 1] && label[p - 1] == blobLabel) {
                    visited[p - 1] = true;
                    queue[tail++] = p - 1;
                }
                if (x < width - 1 && !visited[p + 1] && label[p + 1] == blobLabel) {
                    visited[p + 1] = true;
                    queue[tail++] = p + 1;
                }
                if (y > 0 && !visited[p - width] && label[p - width] == blobLabel) {
                    visited[p - width] = true;
                    queue[tail++] = p - width;
                }
                if (y < height - 1 && !visited[p + width] && label[p + width] == blobLabel) {
                    visited[p + width] = true;
                    queue[tail++] = p + width;
                }
            }
            if (area < MIN_BLOB_PIXELS) {
                continue;
            }
            double pixelX = (double) sumX / area;
            double pixelY = (double) sumY / area;
            double apparentRadius = Math.sqrt(area / Math.PI);
            double radiusMetres = blobLabel == YELLOW ? POLLEN_RADIUS_METRES : NECTAR_RADIUS_METRES;
            double range = focalX * radiusMetres / apparentRadius;
            double bearing = Math.atan2(pixelX - centreX, focalX);
            double elevation = Math.atan2(pixelY - centreY, focalY);
            found.add(new Detection(kindOf(blobLabel), pixelX, pixelY, apparentRadius,
                    range, bearing, elevation));
        }
        return found;
    }

    /**
     * Places a detection in the robot's frame, for the OpMode to drive against.
     *
     * <p>The detector works in the camera's own frame (right, down, along-lens); {@link LensMount}
     * wants (right, along-lens, up), so the vertical axis is negated. The mount also carries the
     * camera's offset from the robot's centre, which is what makes the sighting start at the robot
     * rather than at the lens.</p>
     */
    public static LensMount.Sighting toSighting(Detection detection, LensMount mount) {
        double range = detection.rangeMetres();
        double right = range * Math.tan(detection.bearingRadians());
        double up = -range * Math.tan(detection.elevationRadians());
        return mount.sight(right, range, up);
    }

    private static Kind kindOf(byte label) {
        switch (label) {
            case YELLOW:
                return Kind.POLLEN;
            case RED:
                return Kind.RED_NECTAR;
            case BLUE:
                return Kind.BLUE_NECTAR;
            default:
                throw new IllegalStateException("unlabelled blob reached detection");
        }
    }

    /** Classifies one pixel by hue, after rejecting pixels too grey or too dark to be a ball. */
    private static byte classify(int r, int g, int b) {
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        int delta = max - min;
        if (max == 0 || delta == 0) {
            return NONE;
        }
        double saturation = (double) delta / max;
        double value = max / 255.0;
        if (saturation < SATURATION_MIN || value < VALUE_MIN) {
            return NONE;
        }
        double hue;
        if (max == r) {
            hue = 60.0 * (((double) (g - b) / delta) % 6.0);
        } else if (max == g) {
            hue = 60.0 * (((double) (b - r) / delta) + 2.0);
        } else {
            hue = 60.0 * (((double) (r - g) / delta) + 4.0);
        }
        if (hue < 0.0) {
            hue += 360.0;
        }
        if (hue >= 40.0 && hue <= 75.0) {
            return YELLOW;
        }
        if (hue < 20.0 || hue > 345.0) {
            return RED;
        }
        if (hue >= 200.0 && hue <= 250.0) {
            return BLUE;
        }
        return NONE;
    }
}
