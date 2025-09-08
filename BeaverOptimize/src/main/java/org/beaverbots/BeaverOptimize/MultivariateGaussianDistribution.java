package org.beaverbots.BeaverOptimize;

import java.util.HashSet;
import java.util.Set;
import java.util.stream.IntStream;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.random.Well19937c;

// TODO: Evaluate if it being unused is cause for this class' removal
// TODO: Review AI code
public final class MultivariateGaussianDistribution {
    private final RealVector mean;
    private final RealMatrix covariance;

    // For sampling
    private final RealMatrix choleskyL;
    private final RandomGenerator rng;

    public MultivariateGaussianDistribution(RealVector mean, RealMatrix covariance) {
        this.mean = mean;
        this.covariance = covariance;

        this.choleskyL = new CholeskyDecomposition(covariance).getL();
        this.rng = new Well19937c(); // A good default random generator
    }

    public RealVector getMean() {
        return mean;
    }

    public RealMatrix getCovariance() {
        return covariance;
    }

    public int getDimension() {
        return mean.getDimension();
    }

    public RealVector sample() {
        int dimensions = getDimension();

        RealVector z = new ArrayRealVector(dimensions);
        for (int i = 0; i < dimensions; i++) {
            z.setEntry(i, this.rng.nextGaussian());
        }

        RealVector y = this.choleskyL.operate(z);

        return this.mean.add(y);
    }

    public MultivariateGaussianDistribution condition(
            int[] observedIndices, RealVector observedValues) {
        int[] unobservedIndices = getUnobservedIndices(getDimension(), observedIndices);

        RealVector observedMean = getSubVector(this.mean, observedIndices);
        RealVector unobservedMean = getSubVector(this.mean, unobservedIndices);

        RealMatrix covarianceObservedObserved =
                getSubMatrix(this.covariance, observedIndices, observedIndices);
        RealMatrix covarianceObservedUnobserved =
                getSubMatrix(this.covariance, observedIndices, unobservedIndices);
        RealMatrix covarianceUnobservedObserved =
                getSubMatrix(this.covariance, unobservedIndices, observedIndices);

        RealMatrix covarianceUnobservedUnobserved =
                getSubMatrix(this.covariance, unobservedIndices, unobservedIndices);

        DecompositionSolver solver = new CholeskyDecomposition(covarianceObservedObserved).getSolver();

        RealVector difference = observedValues.subtract(observedMean);
        RealVector meanAdjustment = solver.solve(difference); // This is inv(covarianceObservedObserved) * difference
        RealVector newMean = unobservedMean.add(covarianceUnobservedObserved.operate(meanAdjustment));

        RealMatrix temp = solver.solve(covarianceObservedUnobserved); // This is inv(covarianceObservedObserved) * covarianceObservedUnobserved
        RealMatrix newCovariance = covarianceUnobservedUnobserved.subtract(covarianceUnobservedObserved.multiply(temp));

        return new MultivariateGaussianDistribution(newMean, newCovariance);
    }

    public MultivariateGaussianDistribution marginalize(int[] keptIndices) {
        if (keptIndices.length == getDimension()) {
            return this; // All variables are kept
        }

        RealVector newMean = getSubVector(mean, keptIndices);
        RealMatrix newCovariance = getSubMatrix(covariance, keptIndices, keptIndices);

        return new MultivariateGaussianDistribution(newMean, newCovariance);
    }

    private static int[] getUnobservedIndices(int totalDim, int[] observedIndices) {
        Set<Integer> observedSet = new HashSet<>();
        for (int idx : observedIndices) {
            observedSet.add(idx);
        }
        return IntStream.range(0, totalDim).filter(i -> !observedSet.contains(i)).toArray();
    }

    private static RealVector getSubVector(RealVector vector, int[] indices) {
        RealVector sub = new ArrayRealVector(indices.length);
        for (int i = 0; i < indices.length; i++) {
            sub.setEntry(i, vector.getEntry(indices[i]));
        }
        return sub;
    }

    private static RealMatrix getSubMatrix(RealMatrix matrix, int[] rowIndices, int[] colIndices) {
        RealMatrix sub = new Array2DRowRealMatrix(rowIndices.length, colIndices.length);
        for (int i = 0; i < rowIndices.length; i++) {
            for (int j = 0; j < colIndices.length; j++) {
                sub.setEntry(i, j, matrix.getEntry(rowIndices[i], colIndices[j]));
            }
        }
        return sub;
    }
}