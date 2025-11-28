package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.beaverbots.BeaverCommand.HardwareManager;
import org.beaverbots.BeaverCommand.Subsystem;

public final class Shooter implements Subsystem {
    private double maxPower;

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    private double power;

    public Shooter(double maxPower) {
        this.maxPower = maxPower;

        shooterLeft = HardwareManager.claim(DcMotorEx.class, "shoot");
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight = HardwareManager.claim(DcMotorEx.class, "shoot2");
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public Shooter() {
        this(1);
    }

    public void periodic() {
        shooterLeft.setPower(Math.min(power, maxPower));
        shooterRight.setPower(Math.min(power, maxPower));
    }

    public void spin(double power) {
        this.power = power;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
}
