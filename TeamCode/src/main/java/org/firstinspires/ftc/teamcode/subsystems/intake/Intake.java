package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.BeaverCommand.HardwareManager;
import org.beaverbots.BeaverCommand.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.DrivetrainState;

import java.util.List;

public final class Intake implements Subsystem {
    private double maxPower;

    private DcMotorEx intake;

    private double velocity;

    public Intake(double maxPower) {
        this.maxPower = maxPower;

        this.intake = HardwareManager.claim("intake");
    }

    public Intake() {
        this(1);
    }

    public void periodic() {
        intake.setPower(velocity);
    }

    public void move(double velocity) {
        this.velocity = velocity;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
}
