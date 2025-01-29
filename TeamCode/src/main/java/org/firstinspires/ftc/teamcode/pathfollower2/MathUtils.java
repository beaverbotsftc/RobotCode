package org.firstinspires.ftc.teamcode.pathfollower2;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.function.Function;

public final class MathUtils {
    public static <T> HashSet<HashSet<T>> powerSet(HashSet<T> set) {
        HashSet<HashSet<T>> output = new HashSet<>();

        if (set.isEmpty()) {
            output.add(new HashSet<>());
            return output;
        }

        T element = new ArrayList<T>(set).get(0);
        set.remove(element);

        HashSet<HashSet<T>> subsetsWithoutElement = powerSet(set);

        output.addAll(subsetsWithoutElement);

        for (HashSet<T> subset : subsetsWithoutElement) {
            HashSet<T> newSubset = new HashSet<>(subset);
            newSubset.add(element);
            output.add(newSubset);
        }

        return output;
    }

    public static <T> HashSet<T> compliment(HashSet<T> set, HashSet<T> universal) {
        HashSet<T> output = new HashSet<>();

        for (T element : universal) {
            if (!set.contains(element)) {
                output.add(element);
            }
        }

        return output;
    }

    public static double interpolate(double[] points, double t, double tMax) {
        if (points == null || points.length == 0) {
            throw new IllegalArgumentException("Input array cannot be null or empty.");
        }

        t = t / tMax;

        if (t < 0) t = 0;
        if (t > 1) t = 1;

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

    public static double[] getPoints(Function<Double, Double> f, int points, double tMax) {
        double[] output = new double[points];

        for (int i = 0; i < points; i++) {
            output[i] = f.apply((double) i / points * tMax);
        }

        return output;
    }

    public static double[] evenlySpace(double[] points, int newPoints) { return space(points, newPoints, (Double t) -> t); }

    public static double[] space(double[] points, int newPoints, Function<Double, Double> fSpacing) {
        double[] partialCurveLengths = new double[points.length];
        for (int i = 1; i < points.length; i++) {
            partialCurveLengths[i] = Math.abs(points[i] - points[i - 1]) + partialCurveLengths[i - 1];
        }

        double curveLength = partialCurveLengths[partialCurveLengths.length - 1];

        double[] output = new double[newPoints];
        for (int i = 0; i < newPoints; i++) {
            double t = fSpacing.apply((double) i / (newPoints - 1));
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