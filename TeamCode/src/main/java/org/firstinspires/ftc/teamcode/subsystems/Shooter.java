package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.HardwareManager;
import org.beaverbots.BeaverCommand.Subsystem;
import org.beaverbots.BeaverCommand.util.Stopwatch;
import org.beaverbots.beavertracking.PIDFAxis;

public final class Shooter implements Subsystem {
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private Servo hood;
    private double rpm;
    private PIDFAxis pidf = new PIDFAxis(new PIDFAxis.K(0, 0, 0, 1, 1, 1, 0.1, 0));
    private Stopwatch stopwatch;

    public Shooter() {
        shooterLeft = HardwareManager.claim(DcMotorEx.class, "shoot");
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight = HardwareManager.claim(DcMotorEx.class, "shoot2");
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hood = HardwareManager.claim("hood");

        stopwatch = new Stopwatch();
    }


    public void periodic() {
        double correction = pidf.update(rpm - getVelocity() , rpm / 5140.0,  stopwatch.getDt());
        shooterLeft.setPower(correction);
        shooterRight.setPower(correction);
        RobotLog.i(String.valueOf(getVelocity()));
    }

    public void spin(double rpm) {
        this.rpm = rpm;
    }

    public void setHood(double pos){
        hood.setPosition(pos);
    }

    public double getVelocity(){
        double rpm1 = shooterLeft.getVelocity() / 28.0 * 60.0;
        double rpm2 = shooterRight.getVelocity() / 28.0 * 60.0;
        return (rpm1+rpm2)/2.0;
    }

    public void brakesOn(){
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void brakesOff(){
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
