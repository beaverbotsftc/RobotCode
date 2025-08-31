package org.beaverbots.BeaverOptimize;

import org.apache.commons.math3.linear.RealVector;

public interface Kernel {
    /// Note that the hyperparameters are all in the positive reals, optimized in log space.
    /// If you want unconstrained, take the logarithm.
    double evaluate(RealVector x, RealVector xPrime, RealVector hyperparameters);
}
