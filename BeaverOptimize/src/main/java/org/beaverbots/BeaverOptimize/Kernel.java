package org.beaverbots.BeaverOptimize;

import org.apache.commons.math3.linear.RealVector;

public interface Kernel {
    /// Note that the hyperparameters are all in the positive reals, optimized in log space.
    /// If you want unconstrained, take the logarithm.
    /// Do not use hyperparameter 0
    double evaluate(RealVector x, RealVector xPrime, RealVector hyperparameters);

    ///  Do not include hyperparameter 0
    int getHyperparametersSize();
}
