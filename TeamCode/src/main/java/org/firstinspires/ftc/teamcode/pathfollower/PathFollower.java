package org.firstinspires.ftc.teamcode.pathfollower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.collections.Motors;

public class PathFollower {
    private Telemetry telemetry;

    private double startingTime;
    private double currentTime;
    private Path path;
    private Motors motors;
    private GoBildaPinpointDriver odometry;


    private final double xkR;
    private final double xkPID;
    private final double ykR;
    private final double ykPID;
    private final double thetakR;
    private final double thetakPID;

    private final double xk;
    private final double yk;
    private final double thetak;

    private final double leftFrontk;
    private final double rightFrontk;
    private final double leftBackk;
    private final double rightBackk;

    private PID xPID;
    private PID yPID;
    private PID thetaPID;

    public PathFollower(Telemetry telemetry,
                        long startingTimeNS,
                        Path path,
                        Motors motors,
                        GoBildaPinpointDriver odometry,
                        double xkR,
                        double xkPID,
                        double ykR,
                        double ykPID,
                        double thetakR,
                        double thetakPID,
                        double xkP,
                        double xkI,
                        double xkD,
                        double ykP,
                        double ykI,
                        double ykD,
                        double thetakP,
                        double thetakI,
                        double thetakD,
                        double xk,
                        double yk,
                        double thetak,
                        double leftFrontk,
                        double rightFrontk,
                        double leftBackk,
                        double rightBackk) {
        this.telemetry = telemetry;
        this.startingTime = startingTimeNS * 1e-9;
        this.currentTime = this.startingTime;
        this.path = path;
        this.motors = motors;
        this.odometry = odometry;
        this.xkR = xkR;
        this.xkPID = xkPID;
        this.ykR = ykR;
        this.ykPID = ykPID;
        this.thetakR = thetakR;
        this.thetakPID = thetakPID;
        this.xk = xk;
        this.yk = yk;
        this.thetak = thetak;
        this.leftFrontk = leftFrontk;
        this.rightFrontk = rightFrontk;
        this.leftBackk = leftBackk;
        this.rightBackk = rightBackk;
        this.xPID = new PID(path.x(0) - odometry.getPosition().getX(DistanceUnit.INCH), xkP, xkI, xkD);
        this.yPID = new PID(path.y(0) - odometry.getPosition().getY(DistanceUnit.INCH), ykP, ykI, ykD);
        this.thetaPID = new PID(path.theta(0) - odometry.getPosition().getHeading(AngleUnit.RADIANS), thetakP, thetakI, thetakD);
    }

    private double t() {
        return currentTime - startingTime;
    }

    public void run(long currentTimeNS) {
        double dt = currentTimeNS * 1e-9 - this.currentTime;
        this.currentTime = currentTimeNS * 1e-9;
        telemetry.addData("dt", dt);
        telemetry.addData("currentTimeNS", currentTimeNS);
        telemetry.addData("t()", t());

        telemetry.addData("x", path.x(t()));
        telemetry.addData("y", path.y(t()));
        telemetry.addData("theta", path.theta(t()));

        xPID.update(path.x(t()) - odometry.getPosition().getX(DistanceUnit.INCH), dt);
        yPID.update(path.y(t()) - odometry.getPosition().getY(DistanceUnit.INCH), dt);
        thetaPID.update(path.theta(t()) - odometry.getPosition().getHeading(AngleUnit.DEGREES), dt);

        telemetry.addData("dx", path.dx(t()));
        telemetry.addData("xPID", xPID.correction(path.x(t()) - odometry.getPosition().getX(DistanceUnit.INCH), dt));
        telemetry.addData("dy", path.dy(t()));
        telemetry.addData("yPID", yPID.correction(path.dy(t()) - odometry.getPosition().getY(DistanceUnit.INCH), dt));
        telemetry.addData("dtheta", path.dtheta(t()));
        telemetry.addData("thetaPID", thetaPID.correction(path.theta(t()) - odometry.getPosition().getHeading(AngleUnit.DEGREES), dt));

        double dx = xk * (xkR   * path.dx(t())
                        + xkPID * xPID.correction(path.x(t()) - odometry.getPosition().getX(DistanceUnit.INCH), dt));
        double dy = yk * (ykR   * path.dy(t())
                        - ykPID * yPID.correction(path.y(t()) - odometry.getPosition().getY(DistanceUnit.INCH), dt));
        double dtheta = thetak * (thetakR   * path.dtheta(t())
                                + thetakPID * thetaPID.correction(path.theta(t()) - odometry.getPosition().getHeading(AngleUnit.DEGREES), dt));

        double leftFrontPower  = leftFrontk * (dy + dx + dtheta);
        double rightFrontPower = rightFrontk * (dy - dx - dtheta);
        double leftBackPower   = leftBackk * (dy - dx + dtheta);
        double rightBackPower  = rightBackk * (dy + dx - dtheta);

        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightBackPower", rightBackPower);

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
