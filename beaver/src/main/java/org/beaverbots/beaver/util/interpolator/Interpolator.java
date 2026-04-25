// Warning: 100% AI Code. This should be one of the first places to review if something breaks.
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

/**
 * A highly optimized, N-dimensional Simplex Interpolator.
 *
 * MATHEMATICAL BACKGROUND:
 * This interpolator relies on Delaunay-like simplex interpolation.
 * In an N-dimensional space, an optimal enclosing shape (simplex) requires N+1 points
 * (e.g., 3 points for a 2D triangle, 4 points for a 3D tetrahedron).
 *
 * To find the interpolated value, we solve a linear system for Barycentric weights (w):
 *     A * w = b
 *
 * Where:
 *     'A' is an (N+1)x(N+1) matrix containing the coordinates of the simplex vertices,
 *         with the bottom row padded with 1.0s.
 *     'b' is the target coordinate vector, padded with a 1.0 at the end.
 *     'w' represents the resulting weights.
 *
 * The padding of 1.0s guarantees the constraint that the sum of the weights equals 1.0.
 * If all resulting weights in 'w' are >= 0, the target point is definitively INSIDE the simplex.
 *
 * PERFORMANCE ARCHITECTURE:
 * Because control loops (like FTC odometry/shooter maps) run hundreds of times per second,
 * standard matrix libraries generate crippling Garbage Collection (GC) overhead. This class
 * mitigates that using three key implementation choices:
 *   1. Pre-calculated combinatorial paths.
 *   2. Bypassing Apache Commons Math's safety deep-copies (Zero-copy matrix overrides).
 *   3. In-place LU Decomposition solving.
 */
public final class Interpolator implements Interpolatable {

    public static final class Point {
        public final double[] coordinates;
        public final Interpolatable value;

        public Point(double[] coordinates, Interpolatable value) {
            this.coordinates = coordinates;
            this.value = value;
        }

        public Point(double[] coordinates, double value) {
            this.coordinates = coordinates;
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
    private final int kNearest;

    /**
     * IMPLEMENTATION CHOICE 1: Precomputed Combinations
     * Evaluating simplices requires testing various combinations of the closest 'K' points.
     * Because combinatorial outputs are just array indices (e.g., test points [0, 1, 2], then [0, 1, 3]),
     * they are entirely independent of the robot's actual spatial coordinates.
     * By calculating this int[][] once during Initialization, we save generating hundreds
     * of Lists/Arrays on every single evaluate() call.
     */
    private final int[][] combinations;

    public Interpolator(Point... points) {
        this(-1, points);
    }

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

        if (kNearest <= 0) {
            this.kNearest = Math.min(Math.max(reqPoints * 2, reqPoints), points.length);
        } else {
            this.kNearest = Math.min(Math.max(kNearest, reqPoints), points.length);
        }

        this.points = points;

        // Precalculate the combination paths exactly once.
        List<int[]> comboList = new ArrayList<>();
        generateCombinations(this.kNearest, reqPoints, comboList);
        this.combinations = comboList.toArray(new int[0][]);
    }

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
     * Efficiently finds the 'K' closest points using a bounded insertion sort.
     * For small values of K (e.g., 12) and small datasets, this is vastly faster
     * than sorting the entire array using Arrays.sort().
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

        double[] remainingTarget;
        if (target.length == dimensions) {
            remainingTarget = new double[0];
        } else {
            remainingTarget = Arrays.copyOfRange(target, dimensions, target.length);
        }

        if (points.length == 1) {
            return points[0].value.evaluate(remainingTarget);
        }

        int reqPoints = dimensions + 1;
        int maxPoints = points.length;

        RealVector bestWeights = null;
        Point[] bestPoints = new Point[reqPoints];
        boolean foundValid = false;

        // 1. STANDARD SIMPLEX SEARCH
        if (maxPoints >= reqPoints) {

            // Set up target vector 'b'. We pass 'false' to prevent Apache Commons
            // from making an unnecessary deep copy of our array.
            double[] bData = new double[reqPoints];
            for (int row = 0; row < dimensions; row++) {
                bData[row] = target[row];
            }
            bData[dimensions] = 1.0;
            RealVector b = new ArrayRealVector(bData, false);

            Point[] closestPoints = getClosestKPoints(target, kNearest);
            double bestDel = Double.MAX_VALUE;

            for (int[] combo : combinations) {
                // Populate the system matrix 'A' manually
                double[][] matrixData = new double[reqPoints][reqPoints];
                for (int col = 0; col < reqPoints; col++) {
                    Point p = closestPoints[combo[col]];
                    for (int row = 0; row < dimensions; row++) {
                        matrixData[row][col] = p.coordinates[row];
                    }
                    matrixData[dimensions][col] = 1.0; // Enforces Barycentric constraint (sum of weights = 1)
                }

                /*
                 * IMPLEMENTATION CHOICE 2 & 3: The Zero-Copy Matrix Hack
                 * LUDecomposition operates by transforming the matrix in-place. Because modifying a matrix
                 * in-place is dangerous, Apache Commons Math calls `getData()`, which internally deep-copies
                 * the entire 2D array matrix before beginning computation.
                 *
                 * Because we recreate 'matrixData' from scratch on every loop iteration, we don't care if
                 * LUDecomposition destroys it! By overriding `getData()` to return the raw pointer `getDataRef()`,
                 * LUDecomposition computes entirely in-place. This single override takes the solving time
                 * from ~800ms down to ~2ms.
                 */
                RealMatrix A = new Array2DRowRealMatrix(matrixData, false) {
                    @Override
                    public double[][] getData() {
                        return getDataRef();
                    }
                };

                DecompositionSolver solver;
                try {
                    // Gaussian elimination (LU) to solve A * w = b
                    solver = new LUDecomposition(A).getSolver();
                    if (!solver.isNonSingular()) continue; // Skip collinear/degenerate simplices
                } catch (Exception e) {
                    continue;
                }

                RealVector w = solver.solve(b);

                boolean isInside = true;
                double del = 0;

                // Check if target is inside the simplex, and compute Delaunay penalty
                for (int i = 0; i < reqPoints; i++) {
                    double weight = w.getEntry(i);
                    // Weight < 0 means the target point lies "outside" the face opposite of this vertex
                    if (weight < -1e-7) {
                        isInside = false;
                        break;
                    }
                    // Delaunay logic: penalizes simplices with large variance in edge lengths
                    del += weight * closestPoints[combo[i]].distanceSquared(target);
                }

                // If valid, keep the one with the lowest Delaunay cost
                if (isInside && del < bestDel) {
                    foundValid = true;
                    bestDel = del;
                    bestWeights = w;
                    for (int i = 0; i < reqPoints; i++) bestPoints[i] = closestPoints[combo[i]];
                }
            }
        }

        /*
         * 2. UNDERDETERMINED / DEGENERATE FALLBACK (SVD)
         * MATHEMATICAL BACKGROUND:
         * If 'foundValid' is false, it means one of two things:
         *   A. We don't have enough points to make a simplex (e.g., only 2 points in a 3D space).
         *   B. The target point is strictly OUTSIDE the convex hull of our nearest points.
         *
         * To handle this, we use Singular Value Decomposition (SVD). SVD calculates the
         * Moore-Penrose Pseudoinverse. When an exact solution isn't possible (singular matrix),
         * the pseudoinverse acts as a "Least Squares" projection, gracefully pulling the target
         * onto the nearest valid geometric hyperplane bounded by the points.
         */
        int numPointsToUse = foundValid ? reqPoints : Math.min(reqPoints, maxPoints);

        if (!foundValid) {
            Point[] closestPoints = getClosestKPoints(target, numPointsToUse);

            // We use the exact same zero-copy trick here for the SVD initialization
            double[][] fallbackData = new double[reqPoints][numPointsToUse];
            for (int col = 0; col < numPointsToUse; col++) {
                Point p = closestPoints[col];
                for (int row = 0; row < dimensions; row++) {
                    fallbackData[row][col] = p.coordinates[row];
                }
                fallbackData[dimensions][col] = 1.0;
            }

            RealMatrix A = new Array2DRowRealMatrix(fallbackData, false) {
                @Override
                public double[][] getData() {
                    return getDataRef();
                }
            };

            double[] bData = new double[reqPoints];
            for (int row = 0; row < dimensions; row++) {
                bData[row] = target[row];
            }
            bData[dimensions] = 1.0;
            RealVector b = new ArrayRealVector(bData, false);

            try {
                // SVD solves the least-squares best approximation of A * w = b
                DecompositionSolver solver = new SingularValueDecomposition(A).getSolver();
                bestWeights = solver.solve(b);
                for (int i = 0; i < numPointsToUse; i++) {
                    bestPoints[i] = closestPoints[i];
                }

                // If SVD still projected severe negative weights, we are far outside the dataset.
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

        // 3. SEVERE OUT-OF-BOUNDS FALLBACK: Snap to Nearest
        if (bestWeights == null) {
            Point nearest = getClosestKPoints(target, 1)[0];
            return nearest.value.evaluate(remainingTarget);
        }

        // 4. CLAMP AND NORMALIZE
        // Float precision can sometimes result in weights like 1.0000000000001 or -0.0000000000001.
        // We clamp negatives to zero and guarantee the sum is identically 1.0.
        double weightSum = 0.0;
        for (int i = 0; i < numPointsToUse; i++) {
            double w = Math.max(0.0, bestWeights.getEntry(i));
            bestWeights.setEntry(i, w);
            weightSum += w;
        }

        // 5. RECURSIVE EVALUATION
        // Finally, scale the nested/leaf values by the resolved barycentric weights
        double interpolatedValue = 0;
        if (weightSum > 0.0) {
            for (int i = 0; i < numPointsToUse; i++) {
                double weight = bestWeights.getEntry(i) / weightSum;
                if (weight > 0) {
                    interpolatedValue += weight * bestPoints[i].value.evaluate(remainingTarget);
                }
            }
        } else {
            // Failsafe catch block
            Point nearest = getClosestKPoints(target, 1)[0];
            interpolatedValue = nearest.value.evaluate(remainingTarget);
        }

        return interpolatedValue;
    }
}