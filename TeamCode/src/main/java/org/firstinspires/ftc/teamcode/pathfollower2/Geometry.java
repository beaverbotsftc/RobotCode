package org.firstinspires.ftc.teamcode.pathfollower2;


public final class Geometry {
    public static double interpolate(double[] points, double t) {
        if (points == null || points.length == 0) {
            throw new IllegalArgumentException("Input array cannot be null or empty.");
        }
        if (t < 0 || t > 1) {
            throw new IllegalArgumentException("t must be between 0 and 1 (inclusive).");
        }

        if (points.length == 1) {
            return points[0];
        }

        double index = t * (points.length - 1);
        int lowerIndex = (int) index;
        int upperIndex = lowerIndex + 1;

        // If we would have gotten an OutOfBoundsException (happens if t = 1.0), then
        // return the last value
        if (upperIndex >= points.length) {
            return points[points.length - 1];
        }

        double segmentT = index - lowerIndex;

        return points[lowerIndex] + (points[upperIndex] - points[lowerIndex]) * segmentT;
    }

    public static double[] evenlySpace(double[] points, int newPoints) {
        double[] partialCurveLengths = new double[points.length];
        for (int i = 1; i < points.length; i++) {
            partialCurveLengths[i] = Math.abs(points[i] - points[i - 1]) + partialCurveLengths[i - 1];
        }

        double curveLength = partialCurveLengths[partialCurveLengths.length - 1];

        double[] output = new double[newPoints];
        for (int i = 0; i < newPoints; i++) {
            double t = (double) i / (newPoints - 1);
            double tMapped = t * curveLength;

            for (int j = 1; j < partialCurveLengths.length; j++) {
                if (partialCurveLengths[j] >= tMapped) {
                    double segmentLength = partialCurveLengths[j] - partialCurveLengths[j - 1];
                    double tMappedSubLine = (tMapped - partialCurveLengths[j - 1])
                            / segmentLength;
                    output[i] = (1 - tMappedSubLine) * points[j - 1] + tMappedSubLine * points[j];
                    break;

                }
            }
        }

        return output;
    }
}