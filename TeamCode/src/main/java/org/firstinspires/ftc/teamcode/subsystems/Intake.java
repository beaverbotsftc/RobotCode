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

    // Prioritize high battery autonomous. Also, false negative > false positive.
    private static final int CURRENT_QUEUE_SIZE = 8;
    private static final double CURRENT_CUTOFF_FULL = 3;

    private Queue<Double> currentQueue = new LinkedList<>(Collections.nCopies(CURRENT_QUEUE_SIZE, 0.0));
    private boolean isFull = false;


    private double power;

    public Intake(double maxPower) {
        this.maxPower = maxPower;

        this.intake = HardwareManager.claim("intake");
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Intake() {
        this(1);
    }


    public void periodic() {
        intake.setPower(maxPower * power);

        currentQueue.add(intake.getCurrent(CurrentUnit.AMPS));
        if (currentQueue.size() > CURRENT_QUEUE_SIZE) {
            currentQueue.remove();
        }

        if (power < 0) empty();
    }

    public void spin(double power) {
        this.power = power;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public boolean full() {
        if (isFull) return true;

        for (double current : currentQueue) {
            if (current < CURRENT_CUTOFF_FULL) {
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
