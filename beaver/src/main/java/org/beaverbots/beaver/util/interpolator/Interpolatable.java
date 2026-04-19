package org.beaverbots.beaver.util.interpolator;

/**
 * Functional interface representing a node that can evaluate a target array into a scalar.
 */
@FunctionalInterface
public interface Interpolatable {
    double evaluate(double... target);
}