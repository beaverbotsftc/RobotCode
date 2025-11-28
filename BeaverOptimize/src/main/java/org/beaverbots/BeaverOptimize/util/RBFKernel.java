package org.beaverbots.BeaverOptimize.util;

import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.BeaverOptimize.Kernel;

public class RBFKernel implements Kernel {
    @Override
    public double evaluate(RealVector x1, RealVector x2, RealVector hyperparameters) {
        double signalVariance = hyperparameters.getEntry(0);
        double lengthScale = hyperparameters.getEntry(1);

        double squaredDistance = x1.getDistance(x2);
        squaredDistance *= squaredDistance;

        return signalVariance * Math.exp(-0.5 * squaredDistance / (lengthScale * lengthScale));
    }

    @Override
    public int getHyperparametersSize(int dimension) { return 2; }
}