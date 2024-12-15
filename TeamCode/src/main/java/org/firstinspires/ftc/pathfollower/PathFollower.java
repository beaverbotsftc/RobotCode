package org.firstinspires.ftc.teamcode.pathfollower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.collections.Motors;

public final class PathFollower {
    public static final class RawAndPIDGains {
        public final double r;
        public final double pid;

        public RawAndPIDGains(double r, double pid) {
            this.r = r;
            this.pid = pid;
        }
    }

    public static final class AxisGains {
        public final double x;
        public final double y;
        public final double theta;

        public AxisGains(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }
    }

    public static class MotorGains {
        public final double leftFront;
        public final double rightFront;
        public final double leftBack;
        public final double rightBack;

        public MotorGains(double leftFront, double rightFront, double leftBack, double rightBack) {
            this.leftFront = leftFront;
            this.rightFront = rightFront;
            this.leftBack = leftBack;
            this.rightBack = rightBack;
        }
    }

    private Telemetry telemetry;

    private final double startingTime;
    private double currentTime;
    private final Path path;
    private final Motors motors;
    private final GoBildaPinpointDriver odometry;

    private final PID xPID;
    private final PID yPID;
    private final PID thetaPID;

    private final RawAndPIDGains xRPIDGains;
    private final RawAndPIDGains yRPIDGains;
    private final RawAndPIDGains thetaRPIDGains;
    private final AxisGains axisGains;
    private final MotorGains motorGains;
    private final double speed;

    public PathFollower(Telemetry telemetry,
                        long startingTimeNS,
                        Path path,
                        Motors motors,
                        GoBildaPinpointDriver odometry,
                        RawAndPIDGains xRPIDGains,
                        RawAndPIDGains yRPIDGains,
                        RawAndPIDGains thetaRPIDGains,
                        PIDCoefficients xPIDCoefficients,
                        PIDCoefficients yPIDCoefficients,
                        PIDCoefficients thetaPIDCoefficients,
                        AxisGains axisGains,
                        MotorGains motorGains,
                        double speed
                        ) {
        this.telemetry = telemetry;
        this.startingTime = startingTimeNS * 1e-9;
        this.currentTime = this.startingTime;
        this.path = path;
        this.motors = motors;
        this.odometry = odometry;
        this.xRPIDGains = xRPIDGains;
        this.yRPIDGains = yRPIDGains;
        this.thetaRPIDGains = thetaRPIDGains;
        this.axisGains = axisGains;
        this.motorGains = motorGains;
        this.xPID = new PID(path.x(0) - odometry.getPosition().getX(DistanceUnit.INCH), xPIDCoefficients.p, xPIDCoefficients.i, xPIDCoefficients.d);
        this.yPID = new PID(path.y(0) - odometry.getPosition().getY(DistanceUnit.INCH), yPIDCoefficients.p, yPIDCoefficients.i, yPIDCoefficients.d);
        this.thetaPID = new PID(path.theta(0) - odometry.getPosition().getHeading(AngleUnit.DEGREES), thetaPIDCoefficients.p, thetaPIDCoefficients.i, thetaPIDCoefficients.d);
        this.speed = speed;
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

        double x = odometry.getPosition().getX(DistanceUnit.INCH);
        double y = odometry.getPosition().getY(DistanceUnit.INCH);
        double theta = odometry.getPosition().getHeading(AngleUnit.DEGREES);

        double xError     = path.x(t())     - x;
        double yError     = path.y(t())     - y;
        double thetaError = path.theta(t()) - theta;

        telemetry.addData("xError", xError);
        telemetry.addData("yError", yError);
        telemetry.addData("thetaError", thetaError);

        telemetry.addData("dx", path.dx(t()));
        telemetry.addData("xPID", xPID.correction(xError, dt));
        telemetry.addData("dy", path.dy(t()));
        telemetry.addData("yPID", yPID.correction(yError, dt));
        telemetry.addData("dtheta", path.dtheta(t()));
        telemetry.addData("thetaPID", thetaPID.correction(thetaError, dt));


        double dx = axisGains.x * (xRPIDGains.r   * path.dx(t())
                                 + xRPIDGains.pid * xPID.correction(xError, dt));
        double dy = axisGains.y * (yRPIDGains.r   * path.dy(t())
                                 + yRPIDGains.pid * yPID.correction(yError, dt));
        double dtheta = axisGains.theta * (thetaRPIDGains.r   * path.dtheta(t())
                                             - thetaRPIDGains.pid * thetaPID.correction(thetaError, dt));

        xPID.update(xError, dt);
        yPID.update(yError, dt);
        thetaPID.update(thetaError, dt);

        // Rotates (dx, dy) by -theta to account for the turned wheels
        double theta_rad = theta * Math.PI / 180;
        double rectified_dx = dx * Math.cos(theta_rad) - dy * Math.sin(theta_rad);
        double rectified_dy = dx * Math.sin(theta_rad) + dy * Math.cos(theta_rad);

        telemetry.addData("theta_rad", theta_rad);
        telemetry.addData("rectified_dx", rectified_dx);
        telemetry.addData("rectified_dy", rectified_dy);


        double leftFrontPower  = motorGains.leftFront * (rectified_dx - rectified_dy + dtheta);
        double rightFrontPower = motorGains.rightFront * (rectified_dx + rectified_dy - dtheta);
        double leftBackPower   = motorGains.leftBack * (rectified_dx + rectified_dy + dtheta);
        double rightBackPower  = motorGains.rightBack * (rectified_dx - rectified_dy - dtheta);

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

        motors.leftFrontDrive.setPower(speed * leftFrontPower);
        motors.rightFrontDrive.setPower(speed * rightFrontPower);
        motors.leftBackDrive.setPower(speed * leftBackPower);
        motors.rightBackDrive.setPower(speed * rightBackPower);
    }
}
