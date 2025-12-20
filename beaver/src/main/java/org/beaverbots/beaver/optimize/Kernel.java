package org.beaverbots.beaver.optimize;

import org.apache.commons.math3.linear.RealVector;

public interface Kernel {
    /// The hyperparameters are all in the positive reals, but they are optimized in log space.
    /// If you want unconstrained, take the logarithm.
    double evaluate(RealVector x, RealVector xPrime, RealVector hyperparameters);

    int getHyperparametersSize(int dimension);
}
