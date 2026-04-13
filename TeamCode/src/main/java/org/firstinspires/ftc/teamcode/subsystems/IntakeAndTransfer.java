package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;

public class IntakeAndTransfer implements Subsystem {
    private CachedMotor rightTransfer;
    private CachedMotor leftTransfer;
    private CachedServo stopper;

    private double power = 0;

    public IntakeAndTransfer() {
        leftTransfer = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "transfer2"), 0);
        rightTransfer = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "transfer1"), 0);
        stopper = new CachedServo(HardwareManager.claim(Servo.class, "stopper"), 0);

        leftTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        rightTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intake(double power) {
        this.power = power;
    }

    public void transfer(boolean transfer) {
        stopper.setPosition(transfer ? 0.73 : 0.42);
    }

    public void periodic() {
        leftTransfer.setPower(power);
        rightTransfer.setPower(power);
    }
}
