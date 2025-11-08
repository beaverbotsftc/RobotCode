package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.HardwareManager;
import org.beaverbots.BeaverCommand.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.DrivetrainState;

import java.util.List;

public final class MecanumDrivetrain implements Drivetrain, Subsystem {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DrivetrainState velocity = new DrivetrainState(0, 0, 0);

    private double maxPower;

    public MecanumDrivetrain(double maxPower) {
        this.maxPower = maxPower;

        this.frontLeft = HardwareManager.claim("left front");
        this.frontRight = HardwareManager.claim("right front");
        this.backLeft = HardwareManager.claim("left back");
        this.backRight = HardwareManager.claim("right back");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public MecanumDrivetrain() {
        this(1);
    }

    public void periodic() {
        double frontLeftPower = (velocity.getX() - velocity.getY() - velocity.getTheta()) * Constants.frontLeftDrivetrainMotorPower * Constants.drivetrainPowerConversionFactor;
        double frontRightPower = (velocity.getX() + velocity.getY() + velocity.getTheta()) * Constants.frontRightDrivetrainMotorPower * Constants.drivetrainPowerConversionFactor;
        double backLeftPower = (velocity.getX() + velocity.getY() - velocity.getTheta()) * Constants.backLeftDrivetrainMotorPower * Constants.drivetrainPowerConversionFactor;
        double backRightPower = (velocity.getX() - velocity.getY() + velocity.getTheta()) * Constants.backRightDrivetrainMotorPower * Constants.drivetrainPowerConversionFactor;

        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > this.maxPower) {
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

    public void move(DrivetrainState velocity) {
        this.velocity = velocity;
    }

    public void move(List<Double> velocity) {
        this.velocity = new DrivetrainState(velocity);
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
}
