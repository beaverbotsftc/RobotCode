package org.beaverbots.BeaverSensor;

import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.NonPositiveDefiniteMatrixException;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class UnscentedKalmanFilter {
    final static double beta = 2;

    final int dimensionality;
    final double alpha;
    final double kappa;
    final double lambda;
    final double gamma;
    final double weightMean;
    final double weightCovariance;
    final double weight;

    @FunctionalInterface
    public interface PredictorFunction {
        RealVector apply(RealVector currentState, RealVector controlInput, double dt);
    }

    @FunctionalInterface
    public interface MeasurementFunction {
        RealVector apply(RealVector state);
    }

    final PredictorFunction predictorFunction;

    final RealMatrix processNoise;

    RealVector mean;
    RealMatrix covariance;

    public UnscentedKalmanFilter(int dimensionality, RealVector mean, RealMatrix covariance, double alpha, PredictorFunction predictorFunction, RealMatrix processNoise) {
        this.dimensionality = dimensionality;
        this.alpha = alpha;
        kappa = 0;
        lambda = alpha * alpha * (dimensionality + kappa) - dimensionality;
        gamma = Math.sqrt(dimensionality + lambda);
        weightMean = lambda / (dimensionality + lambda);
        weightCovariance = weightMean + (1 - alpha * alpha + beta);
        weight = 0.5 / (dimensionality + lambda);

        this.predictorFunction = predictorFunction;

        this.processNoise = processNoise;

        this.mean = mean;
        this.covariance = covariance;
    }

    public void predict(RealVector controlVector, double dt) {
        List<RealVector> sigmaPoints = generateSigmaPoints();

        List<RealVector> transformedSigmaPoints = new ArrayList<>();
        for (RealVector sigmaPoint : sigmaPoints) {
            transformedSigmaPoints.add(predictorFunction.apply(sigmaPoint, controlVector, dt));
        }

        RealVector predictedMean = new ArrayRealVector(dimensionality, 0);
        for (int i = 0; i < transformedSigmaPoints.size(); i++) {
            double w = (i == 0) ? weightMean : weight;
            predictedMean = predictedMean.add(transformedSigmaPoints.get(i).mapMultiply(w));
        }

        RealMatrix predictedCovariance = new Array2DRowRealMatrix(dimensionality, dimensionality);
        for (int i = 0; i < transformedSigmaPoints.size(); i++) {
            double w = (i == 0) ? weightCovariance : weight;
            RealVector deviation = transformedSigmaPoints.get(i).subtract(predictedMean);
            predictedCovariance = predictedCovariance.add(deviation.outerProduct(deviation).scalarMultiply(w));
        }

        predictedCovariance = predictedCovariance.add(processNoise);

        this.mean = predictedMean;
        this.covariance = predictedCovariance;
    }

    public void update(RealVector measurement, RealMatrix sensorCovariance, MeasurementFunction measurementFunction) {
        final int measurementDim = measurement.getDimension();

        List<RealVector> sigmaPoints = generateSigmaPoints();

        List<RealVector> measurementPoints = new ArrayList<>();
        for (RealVector point : sigmaPoints) {
            measurementPoints.add(measurementFunction.apply(point));
        }

        RealVector predictedMeasurement = new ArrayRealVector(measurementDim);
        for (int i = 0; i < measurementPoints.size(); i++) {
            double w = (i == 0) ? weightMean : weight;
            predictedMeasurement = predictedMeasurement.add(measurementPoints.get(i).mapMultiply(w));
        }

        RealMatrix innovationCovariance = new Array2DRowRealMatrix(measurementDim, measurementDim);
        for (int i = 0; i < measurementPoints.size(); i++) {
            double w = (i == 0) ? weightCovariance : weight;
            RealVector deviation = measurementPoints.get(i).subtract(predictedMeasurement);
            innovationCovariance = innovationCovariance.add(deviation.outerProduct(deviation).scalarMultiply(w));
        }
        innovationCovariance = innovationCovariance.add(sensorCovariance);

        RealMatrix crossCovariance = new Array2DRowRealMatrix(dimensionality, measurementDim);
        for (int i = 0; i < sigmaPoints.size(); i++) {
            double w = (i == 0) ? weightCovariance : weight;
            RealVector stateDeviation = sigmaPoints.get(i).subtract(mean);
            RealVector measurementDeviation = measurementPoints.get(i).subtract(predictedMeasurement);
            crossCovariance = crossCovariance.add(stateDeviation.outerProduct(measurementDeviation).scalarMultiply(w));
        }

        RealMatrix kalmanGain = crossCovariance.multiply(choleskyDecomposeSafe(innovationCovariance).getSolver().getInverse());

        RealVector innovation = measurement.subtract(predictedMeasurement);
        mean = mean.add(kalmanGain.operate(innovation));
        covariance = enforceSymmetry(covariance.subtract(kalmanGain.multiply(innovationCovariance).multiply(kalmanGain.transpose())));
    }

    public RealVector getMean() {
        return mean.copy();
    }

    public RealMatrix getCovariance() {
        return covariance.copy();
    }

    private List<RealVector> generateSigmaPoints() {
        RealMatrix covarianceSqrt = matrixSqrt(covariance);

        List<RealVector> sigmaPoints = new ArrayList<>(Collections.singletonList(mean));


        for (int i = 0; i < dimensionality; i++) {
            RealVector spreadVector = covarianceSqrt.getColumnVector(i).mapMultiply(gamma);

            sigmaPoints.add(mean.add(spreadVector));
            sigmaPoints.add(mean.subtract(spreadVector));
        }

        return sigmaPoints;
    }

    private RealMatrix matrixSqrt(RealMatrix matrix) {
            return choleskyDecomposeSafe(matrix).getL();
    }

    private RealMatrix enforceSymmetry(RealMatrix matrix) {
        return matrix.add(matrix.transpose()).scalarMultiply(0.5);
    }

    private CholeskyDecomposition choleskyDecomposeSafe(RealMatrix matrix) {
        try {
            return new CholeskyDecomposition(matrix);
        } catch (NonPositiveDefiniteMatrixException e) {
            RobotLog.ee("BeaverSensor", e, "Cholesky decomposition failed, attempting recovery.");
            for (int i = 0; i < 10; i++) {
                try {
                    return new CholeskyDecomposition(matrix.add(MatrixUtils.createRealIdentityMatrix(matrix.getRowDimension()).scalarMultiply(Math.pow(10, i) * 1e-10)));
                } catch (NonPositiveDefiniteMatrixException e2) {
                    RobotLog.ee("BeaverSensor", e2, "Cholesky decomposition failed with correction: 1e-10 * 10^" + i + ", attempting recovery.");
                }
            }
            throw e;
        }
    }
}
