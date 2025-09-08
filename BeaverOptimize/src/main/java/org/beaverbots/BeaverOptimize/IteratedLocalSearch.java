package org.beaverbots.BeaverOptimize;

import android.util.Pair;

import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.random.Well19937c;

import java.util.function.ToDoubleFunction;

public class IteratedLocalSearch {
    public static Pair<RealVector, Double> optimize(
            RealVector initialParams,
            ToDoubleFunction<RealVector> lossFunction,
            double learningRate,
            int localSteps,
            int globalIterations,
            RealVector variance,
            double exponent,
            double maxLossGradient
    ) {
        RandomGenerator rng = new Well19937c();

        Pair<RealVector, Double> bestResult = GradientDescent.optimize(initialParams, lossFunction, learningRate, localSteps, maxLossGradient);

        for (int i = 0; i < globalIterations; i++) {
            try {
                RealVector delta = new ArrayRealVector(bestResult.first.getDimension());
                double perturbationStrength = Math.pow((double) (globalIterations - i) / globalIterations, exponent);
                for (int d = 0; d < bestResult.first.getDimension(); d++) {
                    delta.setEntry(d, rng.nextGaussian() * variance.getEntry(d) * perturbationStrength);
                }

                RealVector perturbedStartPoint = bestResult.first.copy().add(delta);

                Pair<RealVector, Double> proposal = GradientDescent.optimize(
                        perturbedStartPoint, lossFunction, learningRate, localSteps, maxLossGradient
                );

                if (proposal.second < bestResult.second) {
                    bestResult = proposal;
                }
            } catch (Exception e) {
                RobotLog.ee("BeaverOptimize", e, "IteratedLocalSearch: A local optimization step failed. Continuing.");
            }
        }

        return bestResult;
    }
}
