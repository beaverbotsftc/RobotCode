package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.List;

public final class MecanumDrivetrain implements Drivetrain, Subsystem {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DrivetrainState velocity = new DrivetrainState(0, 0, 0);

    private double maxPower;

    private boolean isBraking = true;

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

        setBrake(isBraking);
    }

    public MecanumDrivetrain() {
        this(1);
    }

    List<Double> powerPercentages = new ArrayList<>(List.of(0.0));

    public void periodic() {
        // TODO: stuff
        // TODO: Doesn't work. ?
        if (!NetworkConnectionHandler.getInstance().isNetworkConnected()) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return;
        }

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

        powerPercentages.add(
                Math.max(
                        Math.abs(frontLeft.getVelocity() / 384.5 * 60),
                        Math.max(
                                Math.abs(frontRight.getVelocity() / 384.5 * 60),
                                Math.max(
                                        Math.abs(backLeft.getVelocity() / 384.5 * 60),
                                        Math.abs(backRight.getVelocity() / 384.5 * 60)
                                ))
                ) / 435.0 * 100
        );
    }

    public double getTop10PercentPowerPercentage() {
        powerPercentages.sort(Double::compare);
        return powerPercentages.get((int) (powerPercentages.size() * 0.9));
    }

    public double getTop1PercentPowerPercentage() {
        powerPercentages.sort(Double::compare);
        return powerPercentages.get((int) (powerPercentages.size() * 0.99));
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

    public void setBrake(boolean brake) {
        isBraking = brake;
        if (brake) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void toggleBrake() {
        setBrake(!isBraking);
    }
}
