package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;
import java.util.function.DoubleUnaryOperator;

public final class Transform {
    private double x;
    private double y;
    private double theta;

    public Transform(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Transform(double x, double y) {
        this(x, y, 0);
    }

    public Transform(Pose2D pose) {
        x = pose.getX(DistanceUnit.INCH);
        y = pose.getY(DistanceUnit.INCH);
        theta = pose.getHeading(AngleUnit.RADIANS);
    }

    public Transform(Pose2D pose, double theta) {
        x = pose.getX(DistanceUnit.INCH);
        y = pose.getY(DistanceUnit.INCH);
        this.theta = theta;
    }

    public Transform(double[] state) {
        if (state.length == 3) {
            x = state[0];
            y = state[1];
            theta = state[2];
        } else if (state.length == 2) {
            // TODO: Idk why I did this in the past, seems wrong but I guess it works maybe? I don't think this branch is used.
            x = state[0];
            y = 0;
            theta = state[1];
        }
    }

    public Transform(List<Double> state) {
        this(state.stream().mapToDouble(Double::doubleValue).toArray());
    }

    public Transform(RealVector state) {
        this(state.toArray());
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public Pose2D toPose2d() {
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, theta);
    }

    public double[] toArray() {
        return new double[]{x, y, theta};
    }

    public List<Double> toList() {
        return List.of(x, y, theta);
    }

    public RealVector toVector() {
        return new ArrayRealVector(toArray());
    }

    public Transform rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);

        double newX = this.x * cos - this.y * sin;
        double newY = this.x * sin + this.y * cos;

        return new Transform(newX, newY, this.theta);
    }

    public Transform toLocal(Transform reference) {
        // 1. Translation
        double deltaX = this.x - reference.getX();
        double deltaY = this.y - reference.getY();

        // 2. Rotation
        // We rotate by negative reference theta to convert Global -> Local
        Transform rotated = new Transform(deltaX, deltaY, 0).rotate(-reference.getTheta());

        // 3. Heading Delta
        return new Transform(rotated.getX(), rotated.getY(), this.theta - reference.getTheta());
    }

    public Transform toLocalVelocity(Transform reference) {
        // We simply rotate the current vector by negative reference theta
        return this.rotate(-reference.getTheta());
    }

    public double angleTo(Transform other) {
        double dx = other.getX() - this.x;
        double dy = other.getY() - this.y;
        return Math.atan2(dy, dx);
    }

    public double relativeAngleTo(Transform other) {
        double dx = other.getX() - x;
        double dy = other.getY() - y;
        return Math.atan2(dy, dx) - theta;
    }

    @NonNull
    public String toString() {
        return String.format("x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
    }

    public double lateralDistance(Transform other) {
        return Math.sqrt(Math.pow(x - other.getX(), 2) + Math.pow(y - other.getY(), 2));
    }

    public double angularDistance(Transform other) {
        return Math.abs(theta - other.getTheta());
    }

    public Transform transform(List<DoubleUnaryOperator> f) {
        return new Transform(f.get(0).applyAsDouble(x), f.get(1).applyAsDouble(y), f.get(2).applyAsDouble(theta));
    }
}
