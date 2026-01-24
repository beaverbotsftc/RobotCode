package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;

public final class Intake implements Subsystem {
    private double maxPower;

    private DcMotorEx intake;

    private VoltageSensor voltageSensor;

    private static final int POWER_QUEUE_SIZE= 15;
    private static final double POWER_CUTOFF_FULL = 45;

    private Queue<Double> powerQueue = new LinkedList<>(Collections.nCopies(POWER_QUEUE_SIZE, 0.0));
    private boolean isFull = false;


    private double power;

    public Intake(double maxPower, VoltageSensor voltageSensor) {
        this.maxPower = maxPower;

        this.intake = HardwareManager.claim("intake");
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.voltageSensor = voltageSensor;
    }

    public Intake(VoltageSensor voltageSensor) {
        this(1, voltageSensor);
    }


    public void periodic() {
        intake.setPower(maxPower * power);

        powerQueue.add(intake.getCurrent(CurrentUnit.AMPS) * voltageSensor.getVoltage());
        if (powerQueue.size() > POWER_QUEUE_SIZE) {
            powerQueue.remove();
        }
    }

    public void spin(double power) {
        this.power = power;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public boolean full() {
        if (isFull) return true;

        for (double power : powerQueue) {
            if (power < POWER_CUTOFF_FULL) {
                return false;
            }
        }

        isFull = true;

        return true;
    }

    public void empty() {
        isFull = false;
    }

    public double current() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }
}
