package org.firstinspires.ftc.teamcode.beaverbotv0;

import java.util.Arrays;

public final class MathUtils {
    private MathUtils() {}

    public static double easePolynomial(double start, double end, double degree, double t) {
        // Clamp t to the [0, 1] range
        t = Math.max(0, Math.min(1, t));
        double range = end - start;
        if (t <= 0.5) {
            return start + range / 2.0 * Math.pow(t * 2.0, degree);
        } else {
            return start + range / 2.0 * (2.0 - Math.pow(2.0 - 2.0 * t, degree));
        }
    }

    // TODO: Use De Casteljau's algorithm
    public static double bezier(double[] points, double t) {
        if (points.length == 1) {
            return points[0];
        }
        double[] p1 = Arrays.copyOfRange(points, 0, points.length - 1);
        double[] p2 = Arrays.copyOfRange(points, 1, points.length);
        return (1 - t) * bezier(p1, t) + t * bezier(p2, t);
    }

    public static double interpolate(double[] points, double t) {
        if (points.length == 1) {
            return points[0];
        }

        // Clamp t to [0, 1]
        t = Math.max(0, Math.min(1, t));

        double index = t * (points.length - 1);
        int lowerIndex = (int) index;
        int upperIndex = lowerIndex + 1;

        if (upperIndex >= points.length) {
            return points[points.length - 1];
        }

        double segmentT = index - lowerIndex;
        return points[lowerIndex] + (points[upperIndex] - points[lowerIndex]) * segmentT;
    }

    public static double[] prepend(double value, double[] array) {
        double[] newArray = new double[array.length + 1];
        System.arraycopy(array, 0, newArray, 1, array.length);
        newArray[0] = value;
        return newArray;
    }
}
