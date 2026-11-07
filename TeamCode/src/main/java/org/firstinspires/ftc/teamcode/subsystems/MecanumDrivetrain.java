package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.beaver.cachedhardware.CachedMotor;
import org.firstinspires.ftc.teamcode.beaver.command.HardwareManager;
import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.beaver.util.Transform;

public class MecanumDrivetrain implements Subsystem {
    private CachedMotor frontLeft;
    private CachedMotor frontRight;
    private CachedMotor backLeft;
    private CachedMotor backRight;

    private Transform twist = Transform.ZERO;

    public MecanumDrivetrain() {
        frontLeft = new CachedMotor(HardwareManager.claim("front left drive"), 0);
        frontRight = new CachedMotor(HardwareManager.claim("front right drive"), 0);
        backLeft = new CachedMotor(HardwareManager.claim("back left drive"), 0);
        backRight = new CachedMotor(HardwareManager.claim("back right drive"), 0);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move(Transform twist) {
        this.twist = twist;
    }

    @Override
    public void periodic() {
        frontLeft.setPower(0.2 * (twist.dot(Transform.FORWARD_RIGHT) + Transform.LEFT.rotateLateral(Math.toRadians(45)).dot(Transform.FORWARD_RIGHT) * twist.theta));
        frontRight.setPower(0.2 * (twist.dot(Transform.FORWARD_LEFT) + Transform.LEFT.rotateLateral(Math.toRadians(-45)).dot(Transform.FORWARD_LEFT) * twist.theta));
        backLeft.setPower(0.2 * (twist.dot(Transform.FORWARD_LEFT) + Transform.LEFT.rotateLateral(Math.toRadians(135)).dot(Transform.FORWARD_LEFT) * twist.theta));
        backRight.setPower(0.2 * (twist.dot(Transform.FORWARD_RIGHT) + Transform.LEFT.rotateLateral(Math.toRadians(-135)).dot(Transform.FORWARD_RIGHT) * twist.theta));
    }
}
