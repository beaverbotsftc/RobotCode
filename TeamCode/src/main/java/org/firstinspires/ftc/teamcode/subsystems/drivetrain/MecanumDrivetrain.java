package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;

public final class MecanumDrivetrain implements Drivetrain, Subsystem {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private double x;
    private double y;
    private double theta;

    private double maxPower;

    public MecanumDrivetrain(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public void periodic() {
        double frontLeftPower = (y + x + theta) * Constants.frontLeftDrivetrainMotorPower * Constants.drivetrainPowerConversionFactor;
        double frontRightPower = (y - x - theta) * Constants.frontRightDrivetrainMotorPower * Constants.drivetrainPowerConversionFactor;
        double backLeftPower = (y - x + theta) * Constants.backLeftDrivetrainMotorPower * Constants.drivetrainPowerConversionFactor;
        double backRightPower = (y + x - theta) * Constants.backRightDrivetrainMotorPower * Constants.drivetrainPowerConversionFactor;

        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > this.maxPower) {
            RobotLog.e(
                    String.format("Max power exceeded, scaling down. Power sought: %s, power cap: %s", maxPower, this.maxPower)
            );
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;

            frontLeftPower *= this.maxPower;
            frontRightPower *= this.maxPower;
            backLeftPower *= this.maxPower;
            backRightPower *= this.maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void setMotion(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
}
