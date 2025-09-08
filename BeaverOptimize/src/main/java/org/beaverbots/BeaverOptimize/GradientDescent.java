package org.beaverbots.BeaverOptimize;

import android.util.Pair;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import java.util.function.ToDoubleFunction;

public final class GradientDescent {

    private static final double BASE_EPSILON = 1e-8;

    private GradientDescent() {}

    private static double getEpsilon(double x) {
        return BASE_EPSILON * Math.max(1.0, Math.abs(x));
    }

    public static Pair<RealVector, Double> optimize(RealVector initialParameters,
                                                    ToDoubleFunction<RealVector> lossFunction,
                                                    double learningRate,
                                                    int steps, double maxLossGradient) {
        RealVector currentParameters = initialParameters.copy();

        for (int i = 0; i < steps; i++) {
            RealVector gradient = numericalGradient(currentParameters, lossFunction);
            gradient = gradient.map(g -> Math.max(-maxLossGradient, Math.min(g, maxLossGradient)));

            if (!Double.isFinite(gradient.getNorm())) {
                throw new ArithmeticException("Gradient is infinite");
            }

            RealVector step = gradient.mapMultiply(learningRate);

            currentParameters = currentParameters.subtract(step);
        }

        final double initialLoss = lossFunction.applyAsDouble(initialParameters);
        final double currentLoss = lossFunction.applyAsDouble(currentParameters);

        if (currentLoss > initialLoss)
            return new Pair<>(initialParameters, initialLoss);
        return new Pair<>(currentParameters, currentLoss);
    }

    private static RealVector numericalGradient(RealVector parameters, ToDoubleFunction<RealVector> lossFunction) {
        int numDimensions = parameters.getDimension();
        RealVector gradient = new ArrayRealVector(numDimensions);

        for (int i = 0; i < numDimensions; i++) {
            double originalValue = parameters.getEntry(i);
            double epsilon = getEpsilon(originalValue);

            // Calculate loss at p + epsilon
            RealVector paramsPlusEpsilon = parameters.copy();
            paramsPlusEpsilon.addToEntry(i, epsilon);
            double lossPlus = lossFunction.applyAsDouble(paramsPlusEpsilon);

            // Calculate loss at p - epsilon
            RealVector paramsMinusEpsilon = parameters.copy();
            paramsMinusEpsilon.addToEntry(i, -epsilon);
            double lossMinus = lossFunction.applyAsDouble(paramsMinusEpsilon);

            // If loss is not a finite number, this dimension's gradient is set to 0
            if (!Double.isFinite(lossPlus) || !Double.isFinite(lossMinus)) {
                gradient.setEntry(i, 0.0);
                continue; // Skip to the next dimension
            }

            // Central difference formula for the partial derivative
            double partialDerivative = (lossPlus - lossMinus) / (2.0 * epsilon);
            gradient.setEntry(i, partialDerivative);
        }
        return gradient;
    }
}