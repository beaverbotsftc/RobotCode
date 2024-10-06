package org.firstinspires.ftc.teamcode.pathfollower;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.collections.Motors;

public class PathFollower {
    private double startingTime;
    private double currentTime;
    private Path path;
    private Motors motors;
    private GoBildaPinpointDriver odometry;

    private PID xPID;

    public PathFollower(long startingTimeNS, Path path, Motors motors, GoBildaPinpointDriver odometry, double kP, double kI, double kD) {
        this.startingTime = startingTimeNS * 1e-9;
        this.currentTime = this.startingTime;
        this.path = path;
        this.motors = motors;
        this.odometry = odometry;
        this.xPID = new PID(path.x(0) - odometry.getPosition().getX(DistanceUnit.INCH), kP, kI, kD);
    }

    private double t() {
        return currentTime - startingTime;
    }

    private double dx() {
        return path.dx(t());
    }
    private double dy() {
        return path.dy(t());
    }

    public void apply(long currentTime) {
        double dt = currentTime * 1e-9 - this.currentTime;
        this.currentTime = currentTime * 1e-9;
        double xCorrection = xPID.updateAndGetCorrection(path.x(t()) - odometry.getPosition().getX(DistanceUnit.INCH), dt);
        double x = dx() + xCorrection;
        double y = 0;
        double theta = 0;
        double leftFrontPower  = y + x + theta;
        double rightFrontPower = y - x - theta;
        double leftBackPower   = y - x + theta;
        double rightBackPower  = y + x - theta;

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

        motors.leftFrontDrive.setPower(0.25 * leftFrontPower);
        motors.rightFrontDrive.setPower(0.25 * rightFrontPower);
        motors.leftBackDrive.setPower(0.25 * leftBackPower);
        motors.rightBackDrive.setPower(0.25 * rightBackPower);
    }
}
