package org.firstinspires.ftc.teamcode.pathfollower;

import java.util.function.Function;

public class Path {
    private final double epsilon = 0.001;
    Function<Double, Double> _x;
    double x(double t) {
        return _x.apply(t);
    }
    double dx(double t) {
        return (x(t + epsilon) - x(t)) / epsilon;
    }
    public Path(Function<Double, Double> x) {
        this._x = x;
    }
}