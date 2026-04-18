package org.beaverbots.beaver.util;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;

import java.util.Arrays;
import java.util.Comparator;

public final class Interpolator {

    public static final class Point {
        public final double[] coordinates;
        public final double value;

        /**
         * Constructs a Point.
         * <p>
         * <b>Side-effect warning:</b> To avoid defensive copies, the provided
         * {@code coordinates} array is assigned directly. External modifications
         * to this array will mutate the point's coordinates.
         *
         * @param coordinates the coordinates of the point
         * @param value       the value at the point
         */
        public Point(double[] coordinates, double value) {
            this.coordinates = coordinates;
            this.value = value;
        }

        /**
         * Calculates the squared Euclidean distance to a target point.
         * (Squared distance avoids the computational cost of Math.sqrt)
         */
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

    /**
     * Constructs an Interpolator.
     * <p>
     * <b>Side-effect warning:</b> To avoid defensive copies, the provided
     * {@code points} array is assigned directly. External modifications
     * to the array will mutate the interpolator's internal state.
     *
     * @param points the points to interpolate from
     */
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

        // Assigned directly without defensive copying
        this.points = points;
    }

    /**
     * Evaluates the interpolated value at the given target coordinates.
     * <p>
     * <b>Side-effect warning:</b> To avoid allocations and defensive copies,
     * this method sorts the internal {@code points} array in-place. Because the
     * array's order is mutated upon every evaluation, this method is not thread-safe.
     *
     * @param target the target coordinates
     * @return the interpolated value
     */
    public double evaluate(double... target) {
        if (target.length != dimensions) {
            throw new IllegalArgumentException("Target point must have exactly " + dimensions + " dimensions.");
        }

        // Base case: Only 1 point provided
        if (points.length == 1) {
            return points[0].value;
        }

        // 1. Sort the internal points array in-place by Euclidean distance to the target
        Arrays.sort(points, Comparator.comparingDouble(p -> p.distanceSquared(target)));

        // 2. Select the closest D points, where D = N + 1.
        // (If the user provided fewer than N + 1 total points, we use what we have).
        int D = Math.min(dimensions + 1, points.length);

        // 3. Set up the linear system: A * weights = b
        // Matrix A contains the coordinates of our D points, plus a row of 1s.
        RealMatrix A = new Array2DRowRealMatrix(dimensions + 1, D);
        for (int col = 0; col < D; col++) {
            Point p = points[col];
            for (int row = 0; row < dimensions; row++) {
                A.setEntry(row, col, p.coordinates[row]);
            }
            // The constraint that sum of weights = 1
            A.setEntry(dimensions, col, 1.0);
        }

        // Vector b contains the target coordinates, plus the 1.0 constraint at the end
        RealVector b = new ArrayRealVector(dimensions + 1);
        for (int row = 0; row < dimensions; row++) {
            b.setEntry(row, target[row]);
        }
        b.setEntry(dimensions, 1.0);

        // 4. Solve for the weights using SVD
        // SVD is used because it provides the Moore-Penrose pseudo-inverse.
        // If the simplex is degenerate (e.g., points form a flat line in 2D space),
        // SVD safely projects the target via Least-Squares without crashing.
        DecompositionSolver solver = new SingularValueDecomposition(A).getSolver();
        RealVector weights = solver.solve(b);

        // 5. Interpolate the value using the solved weights
        double interpolatedValue = 0;
        for (int i = 0; i < D; i++) {
            interpolatedValue += weights.getEntry(i) * points[i].value;
        }

        return interpolatedValue;
    }
}