package org.beaverbots.BeaverOptimize;

import android.util.Pair;

import com.qualcomm.robotcore.util.RobotLog;

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
    ///  Index 0 is for noise
    ///  Log space
    private RealVector hyperparameters;

    private List<Pair<RealVector, Double>> train = new ArrayList<>();

    private Kernel kernel;

    public GaussianProcess(Kernel kernel) {
        this.kernel = kernel;
        hyperparameters = new ArrayRealVector(1 + kernel.getHyperparametersSize(), 0);
    }

    public void addTrainingPoint(RealVector x, double y) {
        train.add(new Pair<>(x, y));
    }

    public Pair<Double, Double> predict(RealVector xStar) {
        if (train.isEmpty()) {
            throw new IllegalStateException("Cannot make predictions without training data.");
        }

        RealVector expHyperparameters = hyperparameters.map(x -> Math.exp(x));

        double noiseVariance = expHyperparameters.getEntry(0);

        RealMatrix xTrain = getTrainingInputs();
        RealVector yTrain = getTrainingOutputs();
        final int n = train.size();

        RealMatrix kTrain = computeKernelMatrix(xTrain, xTrain, expHyperparameters);
        RealMatrix kNoisy = kTrain.add(
                MatrixUtils.createRealIdentityMatrix(n).scalarMultiply(noiseVariance)
        );

        RealMatrix xStarMatrix = new Array2DRowRealMatrix(1, xStar.getDimension());
        xStarMatrix.setRowVector(0, xStar);
        RealVector kStar = computeKernelMatrix(xTrain, xStarMatrix, expHyperparameters).getColumnVector(0);

        double kStarStar = kernel.evaluate(xStar, xStar, expHyperparameters);

        DecompositionSolver solver = new CholeskyDecomposition(kNoisy).getSolver();

        RealVector alpha = solver.solve(yTrain); // alpha is the pre-computed (K_y)^-1 * y
        double predictedMean = kStar.dotProduct(alpha);

        RealVector v = solver.solve(kStar); // v is the pre-computed (K_y)^-1 * k_*
        double predictedVariance = kStarStar - kStar.dotProduct(v);

        return new Pair<>(predictedMean, predictedVariance);
    }

    public void optimizeHyperparameters(double learningRate, int steps, int iterations, RealVector variance, double exponent, double maxLossGradient) {
        ToDoubleFunction<RealVector> nlml = createNlmlLossFunction();

        Pair<RealVector, Double> result = IteratedLocalSearch.optimize(
                this.hyperparameters,
                nlml,
                learningRate,
                steps,
                iterations,
                variance,
                exponent, maxLossGradient
        );

        this.hyperparameters = result.first;
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