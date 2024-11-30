package org.firstinspires.ftc.teamcode.pathfollower2;

import java.util.HashMap;
import java.util.Objects;
import java.util.function.Function;

public class Path<Dimension> {
    private static final double epsilon = 1e-3;
    public HashMap<Dimension, Function<Double, Double>> fs;
    public double f(Dimension dimension, double t) {
        return Objects.requireNonNull(fs.get(dimension)).apply(t);
    }

    public double df(Dimension dimension, double t) {
        return (f(dimension, t + epsilon) - f(dimension, t)) / epsilon;
    }
}
