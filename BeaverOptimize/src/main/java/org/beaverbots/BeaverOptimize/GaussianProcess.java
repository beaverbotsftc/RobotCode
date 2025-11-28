package org.beaverbots.BeaverOptimize;

import android.util.Pair;

import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.NonPositiveDefiniteMatrixException;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.List;
import java.util.function.ToDoubleFunction;

public class GaussianProcess {
    final static private double NON_POSITIVE_DEFINITE_LOSS = 1e50;

    final static private double JITTER = 1e-6;

    ///  Index 0 is for noise
    ///  Log space
    private RealVector hyperparameters;

    private List<Pair<RealVector, Double>> train = new ArrayList<>();

    private final Kernel kernel;

    // Cache
    private DecompositionSolver solver;
    private RealVector alpha;
    private RealMatrix xTrainCache;
    private RealVector yTrainCache;
    private RealVector kernelHyperparametersCache;
    private boolean isCacheValid = false;


    public GaussianProcess(Kernel kernel, int dimension) {
        this.kernel = kernel;
        hyperparameters = new ArrayRealVector(1 + kernel.getHyperparametersSize(dimension), 0);
    }

    public void addTrainingPoint(RealVector x, double y) {
        train.add(new Pair<>(x, y));
        isCacheValid = false;
    }

    private void updateCache() {
        if (train.isEmpty()) {
            return;
        }

        RealVector expHyperparameters = hyperparameters.map(Math::exp);
        double noiseVariance = expHyperparameters.getEntry(0) + JITTER;
        this.kernelHyperparametersCache = expHyperparameters.getSubVector(1, expHyperparameters.getDimension() - 1);

        int n = train.size();
        this.xTrainCache = getTrainingInputs();
        this.yTrainCache = getTrainingOutputs();


        RealMatrix kTrain = computeKernelMatrix(xTrainCache, xTrainCache, kernelHyperparametersCache);
        RealMatrix kNoisy = kTrain.add(
                MatrixUtils.createRealIdentityMatrix(n).scalarMultiply(noiseVariance)
        );

        this.solver = new CholeskyDecomposition(kNoisy).getSolver();
        this.alpha = solver.solve(yTrainCache);

        this.isCacheValid = true;
    }


    public Pair<Double, Double> predict(RealVector xStar) {
        if (train.isEmpty()) {
            throw new IllegalStateException("Cannot make predictions without training data.");
        }

        if (!isCacheValid) {
            updateCache();
        }

        RealMatrix xStarMatrix = new Array2DRowRealMatrix(1, xStar.getDimension());
        xStarMatrix.setRowVector(0, xStar);
        RealVector kStar = computeKernelMatrix(xTrainCache, xStarMatrix, kernelHyperparametersCache).getColumnVector(0);

        double kStarStar = kernel.evaluate(xStar, xStar, kernelHyperparametersCache);

        double predictedMean = kStar.dotProduct(alpha);
        RealVector v = solver.solve(kStar); // v is (K_y)^-1 * k_*
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
        this.isCacheValid = false;
    }

    public int getHyperparameterDimension() {
        return hyperparameters.getDimension();
    }

    private ToDoubleFunction<RealVector> createNlmlLossFunction() {
        if (this.train.isEmpty()) {
            return (h) -> 0.0;
        }

        final int n = this.train.size();
        final RealMatrix xTrain = getTrainingInputs();
        final RealVector yTrain = getTrainingOutputs();

        return (proposedHyperparameters) -> {
            RealVector expHyperparameters = proposedHyperparameters.map(Math::exp);

            double noiseVariance = expHyperparameters.getEntry(0) + JITTER;
            RealVector kernelHyperparams = expHyperparameters.getSubVector(1, expHyperparameters.getDimension() - 1);

            RealMatrix kTrain = computeKernelMatrix(xTrain, xTrain, kernelHyperparams);
            RealMatrix kNoisy = kTrain.add(
                    MatrixUtils.createRealIdentityMatrix(n).scalarMultiply(noiseVariance)
            );
            CholeskyDecomposition cholesky;
            try {
                cholesky = new CholeskyDecomposition(kNoisy);
            } catch (NonPositiveDefiniteMatrixException e) {
                RobotLog.e(String.format("%s was not positive definite. Returning a large error value.", kNoisy.toString()));
                return NON_POSITIVE_DEFINITE_LOSS;
            }
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