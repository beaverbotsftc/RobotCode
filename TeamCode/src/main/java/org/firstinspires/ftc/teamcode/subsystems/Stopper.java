package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public final class Stopper implements Subsystem {
    private double maxPower = 0.85;
    private DcMotorEx stopper;
    private double power;

    public Stopper() {
        this.stopper = HardwareManager.claim("stopper");
        this.stopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void periodic() {
        stopper.setPower(maxPower * power);
    }

    public void spin(double power) {
        this.power = power;
    }

    public void spinReverse(){ this.power = -maxPower;}

    public void setMaxPower(double powa){maxPower = powa;}

    public void spinForward(){ this.power = maxPower;}

    public void stop() { this.power = 0; }

}
