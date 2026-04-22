package org.beaverbots.beaver.util.interpolator;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
         */
        public Point(double[] coordinates, double value) {
            this.coordinates = coordinates;
            // A leaf ignores remaining target dimensions and just returns its static value
            this.value = remainingTarget -> value;
        }

        /**
         * Calculates squared Euclidean distance.
         * Safely ignores any extra trailing dimensions in the target array
         * (which belong to nested child interpolators).
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
    private final int kNearest;

    /**
     * Constructs an Interpolator utilizing a default number of local simplex points (12).
     */
    public Interpolator(Point... points) {
        this(-1, points);
    }

    /**
     * Constructs an Interpolator using a specific number of local points to test for optimal simplices.
     * @param kNearest the number of closest points to bound the Delaunay search to.
     * @param points the points of the interpolator.
     */
    public Interpolator(int kNearest, Point... points) {
        if (points == null || points.length == 0) {
            throw new IllegalArgumentException("At least one point must be provided.");
        }

        this.dimensions = points[0].coordinates.length;

        for (Point p : points) {
            if (p.coordinates.length != this.dimensions) {
                throw new IllegalArgumentException("All points must have the same number of dimensions.");
            }
        }

        int reqPoints = this.dimensions + 1;

        // Use default of 12 if a strictly positive kNearest is not provided
        if (kNearest <= 0) {
            this.kNearest = Math.min(Math.max(reqPoints * 4, reqPoints), points.length);
        } else {
            // Assure kNearest checks at least enough points to form a simplex, but no more than what we have
            this.kNearest = Math.min(Math.max(kNearest, reqPoints), points.length);
        }

        this.points = points;
    }

    /**
     * Generates all combinations of size 'r' from 'n' elements.
     * Used to test different simplices (triangles/tetrahedrons) to find the optimal enclosing shape.
     */
    private void generateCombinations(int n, int r, List<int[]> result) {
        if (r > n) return;
        int[] combination = new int[r];
        for (int i = 0; i < r; i++) {
            combination[i] = i;
        }
        while (combination[r - 1] < n) {
            result.add(combination.clone());
            int t = r - 1;
            while (t != 0 && combination[t] == n - r + t) {
                t--;
            }
            combination[t]++;
            for (int i = t + 1; i < r; i++) {
                combination[i] = combination[i - 1] + 1;
            }
        }
    }

    /**
     * Helper to quickly find the closest K points to the target.
     * Uses a simple insertion sort bounded to size K for high performance on small sets.
     */
    private Point[] getClosestKPoints(double[] target, int K) {
        Point[] closestPoints = new Point[K];
        double[] closestDistances = new double[K];
        Arrays.fill(closestDistances, Double.MAX_VALUE);

        for (Point p : points) {
            double dist = p.distanceSquared(target);
            if (dist < closestDistances[K - 1]) {
                int i = K - 1;
                while (i > 0 && dist < closestDistances[i - 1]) {
                    closestDistances[i] = closestDistances[i - 1];
                    closestPoints[i] = closestPoints[i - 1];
                    i--;
                }
                closestDistances[i] = dist;
                closestPoints[i] = p;
            }
        }
        return closestPoints;
    }

    @Override
    public double evaluate(double... target) {
        if (target.length < dimensions) {
            throw new IllegalArgumentException("Target point must have at least " + dimensions + " dimensions.");
        }

        // 1. Slice off dimensions meant for nested children
        double[] remainingTarget;
        if (target.length == dimensions) {
            remainingTarget = new double[0];
        } else {
            remainingTarget = Arrays.copyOfRange(target, dimensions, target.length);
        }

        // Base case: Only 1 point provided
        if (points.length == 1) {
            return points[0].value.evaluate(remainingTarget);
        }

        // A simplex in N dimensions requires N+1 points (e.g. 3 points for 2D plane).
        int reqPoints = dimensions + 1;
        int maxPoints = points.length;

        RealVector bestWeights = null;
        Point[] bestPoints = new Point[reqPoints];
        boolean foundValid = false;

        // If we have enough points to form a simplex, run the optimal Delaunay search
        if (maxPoints >= reqPoints) {

            // Set up target vector 'b' for the linear system: A * weights = b
            RealVector b = new ArrayRealVector(reqPoints);
            for (int row = 0; row < dimensions; row++) {
                b.setEntry(row, target[row]);
            }
            b.setEntry(dimensions, 1.0); // Barycentric constraint: weights must sum to 1

            // 2. LOCAL SIMPLEX SEARCH
            // Use the configured number of local points (kNearest) to find an optimal enclosing simplex.
            Point[] closestPoints = getClosestKPoints(target, kNearest);
            List<int[]> combinations = new ArrayList<>();
            generateCombinations(kNearest, reqPoints, combinations);

            double bestDel = Double.MAX_VALUE;

            // Evaluate every possible simplex formed by these K points
            for (int[] combo : combinations) {
                RealMatrix A = new Array2DRowRealMatrix(reqPoints, reqPoints);
                for (int col = 0; col < reqPoints; col++) {
                    Point p = closestPoints[combo[col]];
                    for (int row = 0; row < dimensions; row++) {
                        A.setEntry(row, col, p.coordinates[row]);
                    }
                    A.setEntry(dimensions, col, 1.0);
                }

                // Attempt to solve for Barycentric weights. Skip if points are collinear/degenerate.
                DecompositionSolver solver;
                try {
                    solver = new LUDecomposition(A).getSolver();
                    if (!solver.isNonSingular()) continue;
                } catch (Exception e) {
                    continue;
                }

                RealVector w = solver.solve(b);

                boolean isInside = true;
                double del = 0; // Delaunay score

                for (int i = 0; i < reqPoints; i++) {
                    double weight = w.getEntry(i);
                    if (weight < -1e-7) {
                        isInside = false; // Target is outside this shape
                        break;            // Stop parsing further, we don't care about external shapes
                    }
                    del += weight * closestPoints[combo[i]].distanceSquared(target);
                }

                if (isInside && del < bestDel) {
                    // The target is INSIDE this simplex!
                    foundValid = true;
                    bestDel = del;
                    bestWeights = w;
                    for (int i = 0; i < reqPoints; i++) bestPoints[i] = closestPoints[combo[i]];
                }
            }
        }

        // 3. UNDERDETERMINED / DEGENERATE FALLBACK
        // If we have fewer points than needed for a simplex, or if all simplices were singular
        // (e.g. all points lie on a straight 1D line in a 3D space), use a Least Squares (SVD) projection fallback.
        int numPointsToUse = foundValid ? reqPoints : Math.min(reqPoints, maxPoints);

        if (!foundValid) {
            Point[] closestPoints = getClosestKPoints(target, numPointsToUse);
            RealMatrix A = new Array2DRowRealMatrix(reqPoints, numPointsToUse);
            for (int col = 0; col < numPointsToUse; col++) {
                Point p = closestPoints[col];
                for (int row = 0; row < dimensions; row++) {
                    A.setEntry(row, col, p.coordinates[row]);
                }
                A.setEntry(dimensions, col, 1.0);
            }

            RealVector b = new ArrayRealVector(reqPoints);
            for (int row = 0; row < dimensions; row++) {
                b.setEntry(row, target[row]);
            }
            b.setEntry(dimensions, 1.0);

            try {
                // SVD pseudo-inverse acts as Least Squares for underdetermined sets
                DecompositionSolver solver = new SingularValueDecomposition(A).getSolver();
                bestWeights = solver.solve(b);
                for (int i = 0; i < numPointsToUse; i++) {
                    bestPoints[i] = closestPoints[i];
                }

                // Test if SVD projected negative weights. If it did, the target is
                // outside the degenerate convex bounds. We drop it to trigger fallback.
                for (int i = 0; i < numPointsToUse; i++) {
                    if (bestWeights.getEntry(i) < -1e-7) {
                        bestWeights = null;
                        break;
                    }
                }
            } catch (Exception e) {
                bestWeights = null;
            }
        }

        // 4. OUTSIDE CONVEX BOUNDS FALLBACK: Snap to Nearest
        if (bestWeights == null) {
            Point nearest = getClosestKPoints(target, 1)[0];
            return nearest.value.evaluate(remainingTarget);
        }

        // 5. CLAMP AND NORMALIZE
        double weightSum = 0.0;
        for (int i = 0; i < numPointsToUse; i++) {
            double w = Math.max(0.0, bestWeights.getEntry(i)); // Zero out negative floats
            bestWeights.setEntry(i, w);
            weightSum += w;
        }

        // 6. RECURSIVE EVALUATION
        double interpolatedValue = 0;
        if (weightSum > 0.0) {
            for (int i = 0; i < numPointsToUse; i++) {
                double weight = bestWeights.getEntry(i) / weightSum;

                // Optimization: skip deep nested evaluations if spatial weight is 0
                if (weight > 0) {
                    interpolatedValue += weight * bestPoints[i].value.evaluate(remainingTarget);
                }
            }
        } else {
            // Unlikely final math-safety fallback
            Point nearest = getClosestKPoints(target, 1)[0];
            interpolatedValue = nearest.value.evaluate(remainingTarget);
        }

        return interpolatedValue;
    }
}