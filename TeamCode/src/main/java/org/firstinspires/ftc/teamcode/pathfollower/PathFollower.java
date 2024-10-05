package org.firstinspires.ftc.teamcode.pathfollower;

import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.Motors;

public class PathFollower {
    double startingTime;
    double currentTime;
    Path path;
    Motors motors;
    GoBildaPinpointDriver odometry;

    public PathFollower(long startingTimeNS, Path path, Motors motors, GoBildaPinpointDriver odometry) {
        this.startingTime = startingTimeNS * 1e-9;
        this.path = path;
        this.motors = motors;
        this.odometry = odometry;
    }

    private double t() {
        return currentTime - startingTime;
    }

    private double dx() {
        return path.dx(t());
    }

    public void apply(long currentTime) {
        this.currentTime = currentTime * 1e-9;

        double x = dx();
        double y = 0;
        double theta = 0;

        double leftFrontPower  = x + y + theta;
        double rightFrontPower = x - y - theta;
        double leftBackPower   = x - y + theta;
        double rightBackPower  = x + y - theta;

        double max;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        motors.leftFrontDrive.setPower(0.15 * leftFrontPower);
        motors.rightFrontDrive.setPower(0.15 * rightFrontPower);
        motors.leftBackDrive.setPower(0.15 * leftBackPower);
        motors.rightBackDrive.setPower(0.15 * rightBackPower);
    }
}
