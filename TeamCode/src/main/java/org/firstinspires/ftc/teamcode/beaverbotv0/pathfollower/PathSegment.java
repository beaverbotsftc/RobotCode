package org.firstinspires.ftc.teamcode.beaverbotv0.pathfollower;

import java.util.function.DoublePredicate;
import java.util.function.DoubleUnaryOperator;

public final class PathSegment {
    public enum State {
        INCOMPLETE,
        COMPLETE,
    }

    private DoubleUnaryOperator path;
    private DoublePredicate finished;

    private double time = 0;

    public PathSegment(DoubleUnaryOperator path, DoublePredicate finished) {
        this.path = path;
        this.finished = finished;
    }

    public State tick(double dt) {
        time += dt;
        if (finished.test(time)) return State.COMPLETE;
        return State.INCOMPLETE;
    }

    public double getPosition() {
        return path.applyAsDouble(time);
    }

    public double getVelocity() {
        final double d = 0.0001;
        return (path.applyAsDouble(time + d) - path.applyAsDouble(time)) / d;
    }

    public double getAcceleration() {
        final double d = 0.0001;
        return (path.applyAsDouble(time + 2 * d) - 2 * path.applyAsDouble(time + d) + path.applyAsDouble(time)) / (d * d);
    }
}
