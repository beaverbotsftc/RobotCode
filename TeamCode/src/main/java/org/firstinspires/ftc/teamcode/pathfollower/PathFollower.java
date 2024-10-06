package org.firstinspires.ftc.teamcode.pathfollower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.collections.Motors;

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
    private double dy() {
        return path.dy(t());
    }

    public void apply(long currentTime) {

        this.currentTime = currentTime * 1e-9;

        double x = dx();
        double y = dy();
        double theta = odometry.getPosition().getHeading(AngleUnit.DEGREES);

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

        motors.leftFrontDrive.setPower(0.35 * leftFrontPower);
        motors.rightFrontDrive.setPower(0.35 * rightFrontPower);
        motors.leftBackDrive.setPower(0.35 * leftBackPower);
        motors.rightBackDrive.setPower(0.35 * rightBackPower);
    }
}
