package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public final class Intake implements Subsystem {
    private double maxPower;

    private DcMotorEx intake;

    private DcMotorEx stopper;

    private double power;

    public Intake(double maxPower) {
        this.maxPower = maxPower;

        this.intake = HardwareManager.claim("intake");
    }

    public Intake() {
        this(1);
    }

    public void periodic() {
        intake.setPower(maxPower * power);
    }

    public void spin(double power) {
        this.power = power;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
}
