package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeAndTransfer implements Subsystem {
    private CachedMotor rightTransfer;
    private CachedMotor leftTransfer;
    private CachedServo stopper;

    private double power = 0;

    private double[] a = new double[100];

    public IntakeAndTransfer(int readFrequency, int readOffset1, int readOffset2) {
        leftTransfer = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "transfer2"), 0, readFrequency, readOffset1);
        rightTransfer = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "transfer1"), 0, readFrequency, readOffset2);
        stopper = new CachedServo(HardwareManager.claim(Servo.class, "stopper"), 0);

        leftTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        rightTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public IntakeAndTransfer() {
        this(1, 0, 0);
    }

    public void intake(double power) {
        CommandOpMode.addData("Power", power);
        this.power = power;
    }

    public void transfer(boolean transfer) {
        stopper.setPosition(transfer ? 0.8 : 0.42);
    }

    public void periodic() {
        leftTransfer.setPower(power);
        rightTransfer.setPower(power);

        CommandOpMode.addData("Current", (leftTransfer.getCurrent(CurrentUnit.AMPS) + rightTransfer.getCurrent(CurrentUnit.AMPS)) / 2.0);
        double mean = 0;
        for (double b : a) mean += b;
        mean /= a.length;
        double stddev = 0;
        for (double b : a) stddev += Math.pow(b - mean, 2);
        stddev /= a.length;
        stddev = Math.sqrt(stddev);
        CommandOpMode.addData("Stddev", stddev);
        for (int i = 0; i < a.length - 1; i++) a[i] = a[i + 1];
        a[a.length - 1] = (leftTransfer.getCurrent(CurrentUnit.AMPS) + rightTransfer.getCurrent(CurrentUnit.AMPS)) / 2.0;
    }
}
