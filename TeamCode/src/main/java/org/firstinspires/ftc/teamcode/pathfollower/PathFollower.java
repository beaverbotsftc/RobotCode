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
    private Path path;
    private Motors motors;
    private GoBildaPinpointDriver odometry;

    private PID xPID;
    private PID yPID;
    private PID thetaPID;

    private RawAndPIDGains xRPIDGains;
    private RawAndPIDGains yRPIDGains;
    private RawAndPIDGains thetaRPIDGains;
    private AxisGains axisGains;
    private MotorGains motorGains;

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
                        MotorGains motorGains
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
        this.thetaPID = new PID(path.theta(0) - odometry.getPosition().getHeading(AngleUnit.RADIANS), thetaPIDCoefficients.p, thetaPIDCoefficients.i, thetaPIDCoefficients.d);
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

        double dx = axisGains.x * (xRPIDGains.r   * path.dx(t())
                                 + xRPIDGains.pid * xPID.correction(path.x(t()) - odometry.getPosition().getX(DistanceUnit.INCH), dt));
        double dy = axisGains.y * (yRPIDGains.r   * path.dy(t())
                                 - yRPIDGains.pid * yPID.correction(path.y(t()) - odometry.getPosition().getY(DistanceUnit.INCH), dt));
        double dtheta = axisGains.theta * (thetaRPIDGains.r   * path.dtheta(t())
                                             + thetaRPIDGains.pid * thetaPID.correction(path.theta(t()) - odometry.getPosition().getHeading(AngleUnit.DEGREES), dt));

        double leftFrontPower  = motorGains.leftFront * (dy + dx + dtheta);
        double rightFrontPower = motorGains.rightFront * (dy - dx - dtheta);
        double leftBackPower   = motorGains.leftBack * (dy - dx + dtheta);
        double rightBackPower  = motorGains.rightBack * (dy + dx - dtheta);

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
