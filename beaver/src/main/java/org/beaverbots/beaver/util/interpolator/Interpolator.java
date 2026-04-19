package org.beaverbots.beaver.util.interpolator;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;

import java.util.Arrays;

public final class Interpolator implements Interpolatable {
    public static final class Point {
        public final double[] coordinates;
        public final Interpolatable value;

        /**
         * Constructs a Point that contains a nested child Interpolator.
         */
        public Point(double[] coordinates, Interpolatable value) {
            this.coordinates = coordinates;
            this.value = value;
        }

        /**
         * Constructs a leaf Point that terminates in a constant double value.
         * (Maintains backwards compatibility with standard 1D/2D interpolators).
         */
        public Point(double[] coordinates, double value) {
            this.coordinates = coordinates;
            // A leaf ignores remaining target dimensions and just returns its static value
            this.value = remainingTarget -> value;
        }

        public double distanceSquared(double[] target) {
            double sum = 0;
            for (int i = 0; i < coordinates.length; i++) {
                double diff = coordinates[i] - target[i];
                sum += diff * diff;
            }
            return sum;
        }
    }

    private final Point[] points;
    private final int dimensions;

    public Interpolator(Point... points) {
        if (points == null || points.length == 0) {
            throw new IllegalArgumentException("At least one point must be provided.");
        }

        this.dimensions = points[0].coordinates.length;

        for (Point p : points) {
            if (p.coordinates.length != this.dimensions) {
                throw new IllegalArgumentException("All points must have the same number of dimensions.");
            }
        }
        this.points = points;
    }

    @Override
    public double evaluate(double... target) {
        // We now check for 'at least' because the target array might contain
        // coordinates meant for nested child interpolators.
        if (target.length < dimensions) {
            throw new IllegalArgumentException("Target point must have at least " + dimensions + " dimensions.");
        }

        // 1. Slice off the dimensions meant for children.
        double[] remainingTarget;
        if (target.length == dimensions) {
            remainingTarget = new double[0]; // Fast-path to avoid allocations on leaves
        } else {
            remainingTarget = Arrays.copyOfRange(target, dimensions, target.length);
        }

        // Base case: Only 1 point provided
        if (points.length == 1) {
            return points[0].value.evaluate(remainingTarget);
        }

        // 2. Find the closest D points directly without sorting the entire array.
        int D = Math.min(dimensions + 1, points.length);
        Point[] closestPoints = new Point[D];
        double[] closestDistances = new double[D];
        Arrays.fill(closestDistances, Double.MAX_VALUE);

        for (Point p : points) {
            double dist = p.distanceSquared(target);
            // If this point is closer than the furthest point currently in our top D list
            if (dist < closestDistances[D - 1]) {
                // Insertion sort into our small tracked array
                int i = D - 1;
                while (i > 0 && dist < closestDistances[i - 1]) {
                    closestDistances[i] = closestDistances[i - 1];
                    closestPoints[i] = closestPoints[i - 1];
                    i--;
                }
                closestDistances[i] = dist;
                closestPoints[i] = p;
            }
        }

        // 3. Set up the linear system: A * weights = b
        RealMatrix A = new Array2DRowRealMatrix(dimensions + 1, D);
        for (int col = 0; col < D; col++) {
            Point p = closestPoints[col];
            for (int row = 0; row < dimensions; row++) {
                A.setEntry(row, col, p.coordinates[row]);
            }
            A.setEntry(dimensions, col, 1.0);
        }

        RealVector b = new ArrayRealVector(dimensions + 1);
        for (int row = 0; row < dimensions; row++) {
            b.setEntry(row, target[row]);
        }
        b.setEntry(dimensions, 1.0);

        // 4. Solve for the weights
        DecompositionSolver solver = new SingularValueDecomposition(A).getSolver();
        RealVector weights = solver.solve(b);

        // 4.5 Clamp barycentric weights to prevent extrapolation
        // This effectively bounds the evaluation strictly inside the convex hull (the simplex).
        double weightSum = 0.0;
        for (int i = 0; i < D; i++) {
            double w = Math.max(0.0, weights.getEntry(i)); // Zero out negative weights
            weights.setEntry(i, w);
            weightSum += w;
        }

        // Normalize weights so they sum to exactly 1.0 (Fixes least squares distortion)
        if (weightSum > 0.0) {
            for (int i = 0; i < D; i++) {
                weights.setEntry(i, weights.getEntry(i) / weightSum);
            }
        } else {
            // Extreme extrapolation fallback: if all weights were somehow negative.
            // Just clamp entirely to the absolute nearest neighbor.
            weights.setEntry(0, 1.0);
            for (int i = 1; i < D; i++) {
                weights.setEntry(i, 0.0);
            }
        }

        // 5. Recursively evaluate the child interpolators and sum them using our clamped spatial weights
        double interpolatedValue = 0;
        for (int i = 0; i < D; i++) {
            double weight = weights.getEntry(i);

            // Optimization: skip evaluation branches if the weight is perfectly 0
            if (weight > 0) {
                double childValue = closestPoints[i].value.evaluate(remainingTarget);
                interpolatedValue += weight * childValue;
            }
        }

        return interpolatedValue;
    }
}