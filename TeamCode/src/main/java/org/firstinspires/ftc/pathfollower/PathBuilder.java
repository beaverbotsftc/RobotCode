package org.firstinspires.ftc.teamcode.pathfollower;

import java.util.function.Function;

public class PathBuilder {
    public double t = 0;
    // TODO: use a list of functions at each step to avoid null pointers and otherwise *whole-robot* crashes
    public Function<Double, Double> _x = (Double t) -> 0.0;
    public Function<Double, Double> _y = (Double t) -> 0.0;
    public Function<Double, Double> _theta = (Double t) -> 0.0;

    public double x(double t) {
        return _x.apply(t);
    }
    public double y(double t) {
        return _y.apply(t);
    }
    public double theta(double t) { return _theta.apply(t); }

    public PathBuilder linarTo(double x, double y, double theta, double time) {
        this._x =     (Double t) -> (t > this.t ? ((x     - this.x(t))     / time) * (t - this.t) + this.x(this.t)     : this.x(t));
        this._y =     (Double t) -> (t > this.t ? ((y     - this.y(t))     / time) * (t - this.t) + this.y(this.t)     : this.y(t));
        this._theta = (Double t) -> (t > this.t ? ((theta - this.theta(t)) / time) * (t - this.t) + this.theta(this.t) : this.theta(t));
        this.t = this.t + time;
        return this;
    }

    public PathBuilder linarTo(double x, double y, double time) {
        return this.linarTo(x, y, this.theta(this.t), time);
    }

    public PathBuilder linarTo(double theta, double time) {
        return this.linarTo(this.x(this.t), this.y(this.t), theta, time);
    }

    public PathBuilder wait(double time) {
        return this.linarTo(x(t), y(t), theta(t), time);
    }

    public Path build() {
        return new Path(_x, _y, _theta);
    }

    public PathBuilder() {}
}
