package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.beaverbots.BeaverCommand.HardwareManager;
import org.beaverbots.BeaverCommand.Subsystem;

public final class Stopper implements Subsystem {
    private double maxPower = 0.85;
    private DcMotorEx stopper;
    private double power;

    public Stopper() {
        this.stopper = HardwareManager.claim("stopper");
    }

    public void periodic() {
        stopper.setPower(maxPower * power);
    }

    public void spin(double power) {
        this.power = power;
    }

    public void spinReverse(){ this.power = -maxPower;}

    public void spinForward(){ this.power = maxPower;}

}
