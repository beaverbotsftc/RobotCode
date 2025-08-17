package org.firstinspires.ftc.teamcode.pathfollower2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.function.Function;

public class MathUtils {
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

    public static double easePolynomial(double start, double end, double degree, double t) {
        if (t <= 0.5) {
            double scaledT = t * 2;
            double factor = Math.pow(scaledT, degree);
            return start + (end - start) * factor / 2;
        } else {
            double scaledT = (t - 0.5) * 2;
            double factor = 1 - Math.pow(1 - scaledT, degree);
            return start + (end - start) * (0.5 + factor / 2);
        }
    }

    public static double easeCompoundPolynomial(double start, double end, double initialDegree, double finalDegree, double interpolationDegree, double t) {
        return easePolynomial(easePolynomial(start, end, initialDegree, t), easePolynomial(start, end, finalDegree, t), interpolationDegree, t);
    }

    // Consider De Casteljau's algorithm
    public static double bezier(double[] points, double t) {
        if (points.length == 1) return points[0];

        return (1 - t) * bezier(Arrays.copyOfRange(points, 0, points.length - 1), t) +
                t * bezier(Arrays.copyOfRange(points, 1, points.length), t);
    }

    public static double interpolate(double[] points, double t, double tMax) {
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

    public static double[] evenlySpace(double[] points, int newPoints) {
        return space(points, newPoints, (Double t) -> t);
    }

    public static double[] space(double[] points, int newPoints, Function<Double, Double> fSpacing) {
        double[] spacedPoints = new double[newPoints];

        // Calculate the spacing factor
        double spacingFactor = (double) (points.length - 1) / (newPoints - 1);

        for (int i = 0; i < newPoints; i++) {
            double t = i * spacingFactor;

            int lowerIndex = (int) t;
            int upperIndex = Math.min(lowerIndex + 1, points.length - 1);

            double segmentT = t - lowerIndex;
            double interpolatedValue = points[lowerIndex] + (points[upperIndex] - points[lowerIndex]) * segmentT;

            spacedPoints[i] = fSpacing.apply(interpolatedValue);
        }

        return spacedPoints;
    }
}