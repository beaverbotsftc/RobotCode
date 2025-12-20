package org.beaverbots.beaver.optimize.kernels;

import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.beaver.optimize.Kernel;

public class ARDRBFKernel implements Kernel {
    @Override
    public double evaluate(RealVector x1, RealVector x2, RealVector hypers) {
        // Hyperparameters: [0]=signalVar, [1..D]lengthScales=
        double weightedDistSq = 0;
        for (int i = 0; i < x1.getDimension(); i++) {
            double lengthScale = hypers.getEntry(i + 1);

            if (lengthScale < 1e-9) {
                lengthScale = 1e-9;
            }

            double diff = (x1.getEntry(i) - x2.getEntry(i)) / lengthScale;
            weightedDistSq += diff * diff;
        }

        return hypers.getEntry(0) * Math.exp(-0.5 * weightedDistSq);
    }

    @Override
    public int getHyperparametersSize(int dimension) {
        return dimension + 1; // 1 Signal Variance + 1 Length Scale per dimension
    }
}
