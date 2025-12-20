package org.beaverbots.beaver.optimize;

import android.util.Pair;

import com.qualcomm.robotcore.util.RobotLog;

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

            RealVector paramsPlusEpsilon = parameters.copy();
            paramsPlusEpsilon.addToEntry(i, epsilon);
            double lossPlus = lossFunction.applyAsDouble(paramsPlusEpsilon);

            RealVector paramsMinusEpsilon = parameters.copy();
            paramsMinusEpsilon.addToEntry(i, -epsilon);
            double lossMinus = lossFunction.applyAsDouble(paramsMinusEpsilon);

            double partialDerivative = (lossPlus - lossMinus) / (2.0 * epsilon);
            if (!Double.isFinite(partialDerivative)) { // Includes NaN too
                RobotLog.ee("BeaverOptimize", "Infinite or NaN partial derivative encountered, continuing by setting it to 0.");
                partialDerivative = 0;
            }

            gradient.setEntry(i, partialDerivative);
        }
        return gradient;
    }
}