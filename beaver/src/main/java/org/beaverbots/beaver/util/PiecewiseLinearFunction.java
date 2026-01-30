package org.beaverbots.beaver.util;

import android.util.Pair;
import java.util.*;

public class PiecewiseLinearFunction {

    private final Segment[] segments;

    public PiecewiseLinearFunction(List<Pair<Double, Double>> points) {
        if (points.size() < 2)
            throw new IllegalArgumentException("Need at least 2 points");

        // Copy and sort by X
        List<Pair<Double, Double>> sorted = new ArrayList<>(points);
        sorted.sort(Comparator.comparingDouble(p -> p.first));

        segments = new Segment[sorted.size() - 1];

        for (int i = 0; i < sorted.size() - 1; i++) {
            Pair<Double, Double> p1 = sorted.get(i);
            Pair<Double, Double> p2 = sorted.get(i + 1);

            double x1 = p1.first;
            double y1 = p1.second;
            double x2 = p2.first;
            double y2 = p2.second;

            if (x2 == x1)
                throw new IllegalArgumentException("Duplicate X values are not allowed");

            double slope = (y2 - y1) / (x2 - x1);
            double intercept = y1 - slope * x1;

            segments[i] = new Segment(x1, x2, slope, intercept);
        }
    }

    /** Evaluates the piecewise function at x */
    public double evaluate(double x) {
        // Clamp to domain
        if (x <= segments[0].xStart)
            return segments[0].valueAt(segments[0].xStart);

        if (x >= segments[segments.length - 1].xEnd)
            return segments[segments.length - 1].valueAt(segments[segments.length - 1].xEnd);

        // Binary search for segment
        int low = 0;
        int high = segments.length - 1;

        while (low <= high) {
            int mid = (low + high) >>> 1;
            Segment s = segments[mid];

            if (x < s.xStart) {
                high = mid - 1;
            } else if (x > s.xEnd) {
                low = mid + 1;
            } else {
                return s.valueAt(x);
            }
        }

        // Should never happen
        return 0;
    }

    ///  Precondition: monotonicity
    public double invert(double y) {
        Segment firstSegment = segments[0];
        Segment lastSegment = segments[segments.length - 1];

        double yStart = firstSegment.valueAt(firstSegment.xStart);
        double yEnd = lastSegment.valueAt(lastSegment.xEnd);

        boolean increasing = yEnd >= yStart;

        // Since it's monotonic, we only need to check the global start and end.
        if (increasing) {
            if (y <= yStart) return firstSegment.xStart;
            if (y >= yEnd) return lastSegment.xEnd;
        } else {
            // Decreasing: yStart is the maximum, yEnd is the minimum
            if (y >= yStart) return firstSegment.xStart;
            if (y <= yEnd) return lastSegment.xEnd;
        }

        // Binary Search for the correct segment
        int low = 0;
        int high = segments.length - 1;

        while (low <= high) {
            int mid = (low + high) >>> 1;
            Segment s = segments[mid];

            double sYStart = s.valueAt(s.xStart);
            double sYEnd = s.valueAt(s.xEnd);

            if (increasing) {
                if (y < sYStart) {
                    high = mid - 1;
                } else if (y > sYEnd) {
                    low = mid + 1;
                } else {
                    // Found the segment: y = mx + b  ->  x = (y - b) / m
                    // Handle horizontal line edge case (m=0)
                    if (Math.abs(s.m) < 1e-9) return s.xStart;
                    return (y - s.b) / s.m;
                }
            } else {
                // Decreasing logic: earlier segments have higher Y values
                if (y > sYStart) {
                    high = mid - 1;
                } else if (y < sYEnd) {
                    low = mid + 1;
                } else {
                    // Found the segment
                    if (Math.abs(s.m) < 1e-9) return s.xStart;
                    return (y - s.b) / s.m;
                }
            }
        }

        // Should never happen
        return 0;
    }

    // ============================

    private static class Segment {
        final double xStart;
        final double xEnd;
        final double m;
        final double b;

        Segment(double xStart, double xEnd, double m, double b) {
            this.xStart = xStart;
            this.xEnd = xEnd;
            this.m = m;
            this.b = b;
        }

        double valueAt(double x) {
            return m * x + b;
        }
    }
}
