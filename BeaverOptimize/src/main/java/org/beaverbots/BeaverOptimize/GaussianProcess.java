package org.beaverbots.BeaverOptimize;

import android.util.Pair;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.random.Well19937c;

import java.util.ArrayList;
import java.util.List;
import java.util.function.ToDoubleFunction;

public class GaussianProcess {
    private MultivariateGaussianDistribution distribution = new MultivariateGaussianDistribution(new ArrayRealVector(), new Array2DRowRealMatrix());

    /// index 0 is for noise
    private RealVector hyperparameters = new ArrayRealVector(new double[] { 0 });

    private List<Pair<RealVector, Double>> train = new ArrayList<>();

    private Kernel kernel;

    private RandomGenerator rng;

    public GaussianProcess(Kernel kernel) {
        this.kernel = kernel;
        this.rng = new Well19937c();
    }

    public void addTrainingPoint(RealVector x, double y) {
        train.add(new Pair<>(x, y));
    }

    public void optimizeHyperparameters(double learningRate, int steps, int iterations, double variance, double exponent) {
        RealVector bestHyperparameters = hyperparameters;
        double loss = Double.POSITIVE_INFINITY;

        for (int i = 0; i < iterations; i++) {
            RealVector delta = new ArrayRealVector(bestHyperparameters.getDimension());
            for (int d = 0; d < bestHyperparameters.getDimension(); d++) {
                delta.setEntry(d, rng.nextGaussian() * variance * Math.pow((double) i / iterations, exponent));
            }

            RealVector perturbedHyperparameters = hyperparameters.copy().add(delta);

            RealVector proposedHyperparameters = GradientDescent.optimize(perturbedHyperparameters, createNlmlLossFunction(), learningRate, steps);
        }
    }

    private ToDoubleFunction<RealVector> createNlmlLossFunction() {
        final int n = this.train.size();
        final int inputDim = this.train.get(0).first.getDimension();
        final RealMatrix xTrain = new Array2DRowRealMatrix(n, inputDim);
        final RealVector yTrain = new ArrayRealVector(n);
        for (int i = 0; i < n; i++) {
            xTrain.setRowVector(i, this.train.get(i).first);
            yTrain.setEntry(i, this.train.get(i).second);
        }

        // The returned lambda is the actual loss function.
        return (hyperparamsLogSpace) -> {
            RealVector hyperparams = hyperparamsLogSpace.map(Math::exp);

            double noiseVariance = hyperparams.getEntry(0);

            RealMatrix kTrain = computeKernelMatrix(xTrain, xTrain, hyperparams);
            RealMatrix kNoisy = kTrain.add(
                    MatrixUtils.createRealIdentityMatrix(n).scalarMultiply(noiseVariance)
            );

            CholeskyDecomposition cholesky = new CholeskyDecomposition(kNoisy);
            DecompositionSolver solver = cholesky.getSolver();
            RealMatrix lMatrix = cholesky.getL();

            // Term 1: Data-fit: 0.5 * y^T * K_y^-1 * y
            RealVector alpha = solver.solve(yTrain);
            double dataFitTerm = 0.5 * yTrain.dotProduct(alpha);

            // Term 2: Complexity penalty: 0.5 * log|K_y|
            double logDeterminant = 0;
            for (int i = 0; i < n; i++) {
                logDeterminant += Math.log(lMatrix.getEntry(i, i));
            }
            double complexityPenaltyTerm = logDeterminant;

            return dataFitTerm + complexityPenaltyTerm;
        };
    }


    private RealMatrix computeKernelMatrix(RealMatrix x1, RealMatrix x2, RealVector hyperparameters) {
        int n1 = x1.getRowDimension();
        int n2 = x2.getRowDimension();
        RealMatrix K = new Array2DRowRealMatrix(n1, n2);

        for (int i = 0; i < n1; i++) {
            for (int j = 0; j < n2; j++) {
                double value = kernel.evaluate(x1.getRowVector(i), x2.getRowVector(j), hyperparameters);
                K.setEntry(i, j, value);
            }
        }
        return K;
    }

    private RealMatrix getTrainingInputs() {
        int n = train.size();
        int inputDim = train.get(0).first.getDimension();
        RealMatrix xTrain = new Array2DRowRealMatrix(n, inputDim);
        for (int i = 0; i < n; i++) {
            xTrain.setRowVector(i, train.get(i).first);
        }
        return xTrain;
    }

    private RealVector getTrainingOutputs() {
        int n = train.size();
        RealVector yTrain = new ArrayRealVector(n);
        for (int i = 0; i < n; i++) {
            yTrain.setEntry(i, train.get(i).second);
        }
        return yTrain;
    }

}
