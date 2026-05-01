package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.optimizedhardware.OptimizedMotor;
import org.beaverbots.beaver.optimizedhardware.OptimizedServo;
import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake implements Subsystem {
    private OptimizedMotor rightTransfer;
    private OptimizedMotor leftTransfer;
    private OptimizedServo stopper;

    private double power = 0;

    private double[] a = new double[100];

    public Intake(int readFrequency, int readOffset1, int readOffset2) {
        leftTransfer = new OptimizedMotor(HardwareManager.claim(DcMotorEx.class, "transfer2"), 0.1, readFrequency, readOffset1, 1, 0);
        rightTransfer = new OptimizedMotor(HardwareManager.claim(DcMotorEx.class, "transfer1"), 0.1, readFrequency, readOffset2, 1, 0);
        stopper = new OptimizedServo(HardwareManager.claim(Servo.class, "stopper"), 0.1, 1, 0, 1, 0);

        leftTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        rightTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Intake() {
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

        /*
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
         */
    }
}
