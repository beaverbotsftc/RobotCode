package org.beaverbots.BeaverSensor;

import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.NonPositiveDefiniteMatrixException;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularMatrixException;

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

        if (isValidVector(predictedMean) && isValidMatrix(predictedCovariance)) {
            this.mean = predictedMean;
            this.covariance = predictedCovariance;
        } else {
            RobotLog.ee("BeaverSensor", "Prediction produced NaN/Inf, keeping previous state.");
        }
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

        RealMatrix kalmanGain;
        try {
            // Use safe cholesky here too to prevent crashing on singular innovation matrices
            kalmanGain = crossCovariance.multiply(choleskyDecomposeSafe(innovationCovariance).getSolver().getInverse());
        } catch (Exception e) {
            RobotLog.ee("BeaverSensor", e, "Failed to compute Kalman Gain (innovation inversion failed). Skipping update.");
            return;
        }

        RealVector innovation = measurement.subtract(predictedMeasurement);

        RealVector newMean = mean.add(kalmanGain.operate(innovation));
        RealMatrix newCovariance = enforceSymmetry(covariance.subtract(kalmanGain.multiply(innovationCovariance).multiply(kalmanGain.transpose())));

        if (isValidVector(newMean) && isValidMatrix(newCovariance)) {
            mean = newMean;
            covariance = newCovariance;
        } else {
            RobotLog.ee("BeaverSensor", "Update produced NaN/Inf, skipping.");
        }
    }

    public RealVector getMean() {
        return mean.copy();
    }

    public RealMatrix getCovariance() {
        return covariance.copy();
    }

    private List<RealVector> generateSigmaPoints() {
        if (!isValidMatrix(covariance)) {
            RobotLog.ee("BeaverSensor", "Covariance is NaN/Inf before sigma point generation. Resetting to Identity.");
            covariance = MatrixUtils.createRealIdentityMatrix(dimensionality);
        }

        RealMatrix covarianceSqrt;
        try {
            covarianceSqrt = matrixSqrt(covariance);
        } catch (Exception e) {
            RobotLog.ee("BeaverSensor", e, "Covariance sqrt failed completely. Resetting covariance.");
            covariance = MatrixUtils.createRealIdentityMatrix(dimensionality).scalarMultiply(1.0);
            covarianceSqrt = matrixSqrt(covariance);
        }

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

    private boolean isValidMatrix(RealMatrix m) {
        for (int r = 0; r < m.getRowDimension(); r++) {
            for (int c = 0; c < m.getColumnDimension(); c++) {
                double val = m.getEntry(r, c);
                if (Double.isNaN(val) || Double.isInfinite(val)) return false;
            }
        }
        return true;
    }

    private boolean isValidVector(RealVector v) {
        for (int i = 0; i < v.getDimension(); i++) {
            double val = v.getEntry(i);
            if (Double.isNaN(val) || Double.isInfinite(val)) return false;
        }
        return true;
    }

    private CholeskyDecomposition choleskyDecomposeSafe(RealMatrix matrix) {
        // Force throw if NaN, so we can't accidentally succeed with a NaN L-matrix
        if (!isValidMatrix(matrix)) {
            throw new NonPositiveDefiniteMatrixException(0, 0, 0);
        }

        try {
            return new CholeskyDecomposition(matrix);
        } catch (NonPositiveDefiniteMatrixException e) {
            // RobotLog.ee("BeaverSensor", e, "Cholesky decomposition failed, attempting recovery.");
            for (int i = 0; i < 10; i++) {
                try {
                    // Adjusted jitter calculation
                    double jitter = 1e-9 * Math.pow(10, i * 0.5);
                    return new CholeskyDecomposition(matrix.add(MatrixUtils.createRealIdentityMatrix(matrix.getRowDimension()).scalarMultiply(jitter)));
                } catch (NonPositiveDefiniteMatrixException e2) {
                    // RobotLog.ee("BeaverSensor", e2, "Cholesky decomposition failed with correction: " + i + ", attempting recovery.");
                }
            }
            throw e;
        }
    }
}