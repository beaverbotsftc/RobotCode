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
    private PID yPID;

    public PathFollower(long startingTimeNS, Path path, Motors motors, GoBildaPinpointDriver odometry, double xkP, double xkI, double xkD, double ykP, double ykI, double ykD) {
        this.startingTime = startingTimeNS * 1e-9;
        this.currentTime = this.startingTime;
        this.path = path;
        this.motors = motors;
        this.odometry = odometry;
        this.xPID = new PID(path.x(0) - odometry.getPosition().getX(DistanceUnit.INCH), xkP, xkI, xkD);
        this.yPID = new PID(path.y(0) - odometry.getPosition().getX(DistanceUnit.INCH), ykP, ykI, ykD);
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

    public void run(long currentTime) {
        double dt = currentTime * 1e-9 - this.currentTime;
        this.currentTime = currentTime * 1e-9;
        double x = dx() + xPID.updateAndGetCorrection(path.x(t()) - odometry.getPosition().getX(DistanceUnit.INCH), dt);
        double y = dy() + yPID.updateAndGetCorrection(path.y(t()) - odometry.getPosition().getY(DistanceUnit.INCH), dt);
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
