package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

public final class DrivetrainState {
    private double x;
    private double y;
    private double theta;

    public DrivetrainState(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public DrivetrainState(Pose2D pose) {
        x = pose.getX(DistanceUnit.INCH);
        y = pose.getY(DistanceUnit.INCH);
        theta = pose.getHeading(AngleUnit.RADIANS);
    }

    public DrivetrainState(List<Double> state) {
        x = state.get(0);
        y = state.get(1);
        theta = state.get(2);
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

    public String toString() {
        return String.format("x=%s, y=%s, theta=%s", String.valueOf(x), String.valueOf(y), String.valueOf(theta));
    }
}
