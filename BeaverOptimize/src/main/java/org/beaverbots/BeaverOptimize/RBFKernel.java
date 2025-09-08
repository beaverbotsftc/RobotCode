package org.beaverbots.BeaverOptimize;

import org.apache.commons.math3.linear.RealVector;

public class RBFKernel implements Kernel {
    @Override
    public double evaluate(RealVector x1, RealVector x2, RealVector hyperparameters) {
        double signalVariance = hyperparameters.getEntry(1);
        double lengthScale = hyperparameters.getEntry(2);

        double squaredDistance = x1.getDistance(x2);
        squaredDistance *= squaredDistance;

        return signalVariance * Math.exp(-0.5 * squaredDistance / (lengthScale * lengthScale));
    }

    @Override
    public int getHyperparametersSize() { return 2; }
}