package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.BeaverCommand.HardwareManager;
import org.beaverbots.BeaverCommand.Subsystem;

public final class Shooter implements Subsystem {
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private Servo hood;
    private double rpm;

    public Shooter() {
        shooterLeft = HardwareManager.claim(DcMotorEx.class, "shoot");
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight = HardwareManager.claim(DcMotorEx.class, "shoot2");
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hood = HardwareManager.claim("hood");
    }


    public void periodic() {
        shooterLeft.setPower(rpm/5140.0);
        shooterRight.setPower(rpm/5140.0);
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
}
