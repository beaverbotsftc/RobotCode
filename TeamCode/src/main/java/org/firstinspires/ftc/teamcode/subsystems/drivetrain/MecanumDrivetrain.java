package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

import java.util.ArrayList;
import java.util.List;

public final class MecanumDrivetrain implements Drivetrain, Subsystem {
    private CachedMotor frontLeft;
    private CachedMotor frontRight;
    private CachedMotor backLeft;
    private CachedMotor backRight;

    private Transform velocity = new Transform(0, 0, 0);

    private boolean isBraking = false;

    public MecanumDrivetrain(VoltageSensor voltageSensor) {
        this.frontLeft = new CachedMotor(HardwareManager.claim("front left drive"), 0.01);
        this.frontRight = new CachedMotor(HardwareManager.claim("front right drive"), 0.01);
        this.backLeft = new CachedMotor(HardwareManager.claim("back left drive"), 0.01);
        this.backRight = new CachedMotor(HardwareManager.claim("back right drive"), 0.01);

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

    List<Double> powerPercentages = new ArrayList<>(List.of(0.0));

    public void periodic() {
        // TODO: stuff
        // TODO: Doesn't work. ?
        /*
        if (!NetworkConnectionHandler.getInstance().isNetworkConnected()) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return;
        }
         */

        double frontLeftPower = Constants.drivetrainFrontLeftFactor * (velocity.getX() * Constants.drivetrainPowerConversionFactorX - velocity.getY() * Constants.drivetrainPowerConversionFactorY - velocity.getTheta() * Constants.drivetrainPowerConversionFactorTheta);
        double frontRightPower = Constants.drivetrainFrontRightFactor * (velocity.getX() * Constants.drivetrainPowerConversionFactorX + velocity.getY() * Constants.drivetrainPowerConversionFactorY + velocity.getTheta() * Constants.drivetrainPowerConversionFactorTheta);
        double backLeftPower = Constants.drivetrainBackLeftFactor * (velocity.getX() * Constants.drivetrainPowerConversionFactorX + velocity.getY() * Constants.drivetrainPowerConversionFactorY - velocity.getTheta() * Constants.drivetrainPowerConversionFactorTheta);
        double backRightPower = Constants.drivetrainBackRightFactor * (velocity.getX() * Constants.drivetrainPowerConversionFactorX - velocity.getY() * Constants.drivetrainPowerConversionFactorY + velocity.getTheta() * Constants.drivetrainPowerConversionFactorTheta);

        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // TODO: Profiling says really slow, like 10% of the loop times slow.
        /*
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
         */
    }

    public double getTop10PercentPowerPercentage() {
        powerPercentages.sort(Double::compare);
        return powerPercentages.get((int) (powerPercentages.size() * 0.9));
    }

    public double getTop1PercentPowerPercentage() {
        powerPercentages.sort(Double::compare);
        return powerPercentages.get((int) (powerPercentages.size() * 0.99));
    }

    public void move(Transform force) {
        this.velocity = force;
    }

    public void move(Transform force, Transform position) {
        this.velocity = force.toLocalVelocity(position);
    }

    public void move(List<Double> force, List<Double> position) {
        move(new Transform(force), new Transform(position));
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
