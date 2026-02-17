package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public final class Stopper implements Subsystem {
    private CachedMotor stopper;
    private double power;

    public Stopper() {
        this.stopper = new CachedMotor(HardwareManager.claim("stopper"), 0.01);
        this.stopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void periodic() {
        stopper.setPower(power);
    }

    public void spin(double power) {
        this.power = power;
    }

    public void stop() { this.power = 0; }

}
