package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import androidx.annotation.NonNull;

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

    public DrivetrainState(Pose2D pose, double theta) {
        x = pose.getX(DistanceUnit.INCH);
        y = pose.getY(DistanceUnit.INCH);
        this.theta = theta;
    }

    public DrivetrainState(List<Double> state) {
        if (state.size() == 3) {
            x = state.get(0);
            y = state.get(1);
            theta = state.get(2);
        } else if (state.size() == 2) {
            x = state.get(0);
            y = 0;
            theta = state.get(1);
        }
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

    public DrivetrainState toLocal(DrivetrainState reference) {
        double deltaX = this.x - reference.getX();
        double deltaY = this.y - reference.getY();

        double sinTheta = Math.sin(reference.getTheta());
        double cosTheta = Math.cos(reference.getTheta());

        // Equivalent to a rotation matrix multiplication
        double localX = deltaX * cosTheta + deltaY * sinTheta;
        double localY = -deltaX * sinTheta + deltaY * cosTheta;

        double localTheta = this.theta - reference.getTheta();

        return new DrivetrainState(localX, localY, localTheta);
    }

    public DrivetrainState toLocalVelocity(DrivetrainState position) {
        double sinTheta = Math.sin(position.getTheta());
        double cosTheta = Math.cos(position.getTheta());

        // Equivalent to a rotation matrix multiplication
        double localX = this.x * cosTheta + this.y * sinTheta;
        double localY = -this.x * sinTheta + this.y * cosTheta;

        return new DrivetrainState(localX, localY, this.theta);
    }

    @NonNull
    public String toString() {
        return String.format("x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
    }
}
