package org.beaverbots.BeaverOptimize;

import org.apache.commons.math3.linear.RealVector;

public class ARDRBFKernel implements Kernel {
    @Override
    public double evaluate(RealVector x1, RealVector x2, RealVector logHypers) {
        // Hypers: [0]=LogSignalVar, [1..D]=LogLengthScales
        double signalVar = Math.exp(logHypers.getEntry(0));

        double weightedDistSq = 0;
        for (int i = 0; i < x1.getDimension(); i++) {
            double lengthScale = Math.exp(logHypers.getEntry(i + 1));
            double diff = (x1.getEntry(i) - x2.getEntry(i)) / lengthScale;
            weightedDistSq += diff * diff;
        }

        return signalVar * Math.exp(-0.5 * weightedDistSq);
    }

    @Override
    public int getHyperparametersSize(int dimension) {
        return dimension + 1; // 1 Signal Variance + 1 Length Scale per dimension
    }
}
