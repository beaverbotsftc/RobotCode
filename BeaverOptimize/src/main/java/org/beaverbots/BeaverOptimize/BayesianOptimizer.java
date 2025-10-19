package org.beaverbots.BeaverOptimize;

import android.util.Pair;

import org.apache.commons.math3.distribution.NormalDistribution;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.List;
import java.util.function.ToDoubleFunction;

public class BayesianOptimizer {
    private GaussianProcess gaussianProcess;
    private Pair<RealVector, RealVector> bounds;
    private RealVector middle;
    private List<Pair<RealVector, Double>> observedPoints = new ArrayList<>();
    private double kappa;

    final static private double LEARNING_RATE = 0.001;
    final static private int LOCAL_STEPS = 120;
    final static private int GLOBAL_ITERATIONS = 120;
    final static private double EXPONENT = 2.0;
    final static private double MAX_LOSS_GRADIENT = 1000.0;

    final static private double OUT_OF_BOUNDS_SLOPE = 1e10;
    final static private double OUT_OF_BOUNDS_MINIMUM = 1e10;

    final static private double PERTURBATION_SCALE = 0.3;

    final static private double HYPERPARAMETERS_LEARNING_RATE = 0.01;
    final static private int HYPERPARAMETERS_LOCAL_STEPS = 50;
    final static private int HYPERPARAMETERS_GLOBAL_ITERATIONS = 20;
    final static private double HYPERPARAMETERS_VARIANCE_MULTIPLIER = 1;
    final static private double HYPERPARAMETERS_EXPONENT = 2;
    final static private double HYPERPARAMETERS_MAX_LOSS_GRADIENT = 100;
    final static private double HYPERPARAMETERS_UPDATE_BASE = 1.2;

    public BayesianOptimizer(Kernel kernel, Pair<RealVector, RealVector> bounds, double confidence) {
        this.gaussianProcess = new GaussianProcess(kernel, bounds.first.getDimension());
        this.bounds = bounds;
        this.middle = bounds.first.add(bounds.second).mapDivide(2);
        setConfidence(confidence);
    }

    public void setConfidence(double confidence) {
        kappa = new NormalDistribution(0, 1).inverseCumulativeProbability(confidence);
    }

    public Pair<RealVector, Double> getBestObservedPoint() {
        if (observedPoints.isEmpty()) throw new IllegalStateException("No points observed yet.");
        Pair<RealVector, Double> best = observedPoints.get(0);
        for (int i = 1; i < observedPoints.size(); i++) {
            if (observedPoints.get(i).second < best.second) {
                best = observedPoints.get(i);
            }
        }
        return best;
    }

    public void addObservedPoint(RealVector x, double y) {
        observedPoints.add(new Pair<>(x, y));
        gaussianProcess.addTrainingPoint(x, y);
        if (
                observedPoints.size() != 1 &&
                (int) (Math.log(observedPoints.size()) / Math.log(HYPERPARAMETERS_UPDATE_BASE))
                != (int) (Math.log(observedPoints.size() - 1) / Math.log(HYPERPARAMETERS_UPDATE_BASE))) {
            gaussianProcess.optimizeHyperparameters(
                    HYPERPARAMETERS_LEARNING_RATE,
                    HYPERPARAMETERS_LOCAL_STEPS,
                    HYPERPARAMETERS_GLOBAL_ITERATIONS,
                    new ArrayRealVector(
                            gaussianProcess.getHyperparameterDimension(),
                            HYPERPARAMETERS_VARIANCE_MULTIPLIER),
                    HYPERPARAMETERS_EXPONENT,
                    HYPERPARAMETERS_MAX_LOSS_GRADIENT);
        }
    }

    public RealVector findNextPoint() {
        if (observedPoints.isEmpty()) {
            return bounds.first.add(bounds.second).mapDivide(2);
        }

        ToDoubleFunction<RealVector> lcb = x -> {
            double outOfBoundsPenalty = 0;
            for (int i = 0; i < x.getDimension(); i++) {
                if (x.getEntry(i) < bounds.first.getEntry(i)) {
                    outOfBoundsPenalty += OUT_OF_BOUNDS_SLOPE * (bounds.first.getEntry(i) - x.getEntry(i));
                } else if (x.getEntry(i) > bounds.second.getEntry(i)) {
                    outOfBoundsPenalty += OUT_OF_BOUNDS_SLOPE * (x.getEntry(i) - bounds.second.getEntry(i));
                }
            }
            if (outOfBoundsPenalty != 0) return outOfBoundsPenalty + OUT_OF_BOUNDS_MINIMUM;

            Pair<Double, Double> prediction = gaussianProcess.predict(x);
            double mu = prediction.first;
            double variance = prediction.second;
            double sigma = Math.sqrt(Math.max(variance, 1e-9));

            // We minimize the LCB.
            return mu - kappa * sigma;
        };

        RealVector initialGuess = middle.copy();

        // The variance for the perturbation should be a fraction of the total search space size.
        RealVector variance = bounds.second.subtract(bounds.first).mapMultiply(PERTURBATION_SCALE);

        Pair<RealVector, Double> result = IteratedLocalSearch.optimize(
                initialGuess,
                lcb,
                LEARNING_RATE,
                LOCAL_STEPS,
                GLOBAL_ITERATIONS,
                variance,
                EXPONENT,
                MAX_LOSS_GRADIENT
        );

        RealVector nextX = result.first.copy();
        for (int i = 0; i < nextX.getDimension(); i++) {
            nextX.setEntry(i, Math.min(bounds.second.getEntry(i), Math.max(nextX.getEntry(i), bounds.first.getEntry(i))));
        }

        return nextX;
    }
}