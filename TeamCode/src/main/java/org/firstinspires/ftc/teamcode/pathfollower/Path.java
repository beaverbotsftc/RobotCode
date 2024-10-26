package org.firstinspires.ftc.teamcode.pathfollower;

import java.util.function.Function;

public class Path {
    private final double epsilon = 0.001;
    Function<Double, Double> _x;
    Function<Double, Double> _y;
    Function<Double, Double> _theta;
    double x(double t) {
        return _x.apply(t);
    }
    double y(double t) {
        return _y.apply(t);
    }
    double theta(double t) { return _theta.apply(t); }
    double dx(double t) {
        return (x(t + epsilon) - x(t)) / epsilon;
    }
    double dy(double t) { return (y(t + epsilon) - y(t)) / epsilon; }
    double dtheta(double t) { return (theta(t + epsilon) - theta(t)) / epsilon; }
    public Path(Function<Double, Double> x, Function<Double, Double> y, Function<Double, Double> theta) {
        this._x = x;
        this._y = y;
        this._theta = theta;
    }
}