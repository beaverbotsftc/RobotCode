package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.BeaverCommand.HardwareManager;
import org.beaverbots.BeaverCommand.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;

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

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */
    }

    public MecanumDrivetrain() {
        this(1);
    }

    public void periodic() {
        double frontLeftPower = (velocity.getX() * Constants.drivetrainPowerConversionFactorX - velocity.getY() * Constants.drivetrainPowerConversionFactorY - velocity.getTheta() * Constants.drivetrainPowerConversionFactorTheta);
        double frontRightPower = (velocity.getX() * Constants.drivetrainPowerConversionFactorX + velocity.getY() * Constants.drivetrainPowerConversionFactorY + velocity.getTheta() * Constants.drivetrainPowerConversionFactorTheta);
        double backLeftPower = (velocity.getX() * Constants.drivetrainPowerConversionFactorX + velocity.getY() * Constants.drivetrainPowerConversionFactorY - velocity.getTheta() * Constants.drivetrainPowerConversionFactorTheta);
        double backRightPower = (velocity.getX() * Constants.drivetrainPowerConversionFactorX - velocity.getY() * Constants.drivetrainPowerConversionFactorY + velocity.getTheta() * Constants.drivetrainPowerConversionFactorTheta);

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

    public void move(DrivetrainState velocity, DrivetrainState position) {
        this.velocity = velocity.toLocalVelocity(position);
    }

    public void move(List<Double> velocity, List<Double> position) {
        move(new DrivetrainState(velocity), new DrivetrainState(position));
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
}
