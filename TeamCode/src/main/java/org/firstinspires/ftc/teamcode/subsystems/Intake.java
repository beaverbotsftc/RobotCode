package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;

public final class Intake implements Subsystem {
    private double maxPower;

    private CachedMotor intake;

    // Prioritize high battery autonomous. Also, false negative > false positive.
    private static final int CURRENT_QUEUE_SIZE = 12;
    private static final double CURRENT_CUTOFF_FULL = 4.5;
    private static final double CURRENT_CUTOFF_EMPTY = 2.6;

    private Queue<Double> currentQueue = new LinkedList<>(Collections.nCopies(CURRENT_QUEUE_SIZE, 0.0));


    private double power;

    public Intake(double maxPower) {
        this.maxPower = maxPower;

        this.intake = new CachedMotor(HardwareManager.claim("intake"), 0.01);
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Intake() {
        this(1);
    }


    public void periodic() {
        intake.setPower(maxPower * power);

        if (power != 0)
            currentQueue.add(current());
        else
            currentQueue.add(Double.NaN);

        if (currentQueue.size() > CURRENT_QUEUE_SIZE) {
            currentQueue.remove();
        }

        RobotLog.dd("Intake", Arrays.toString(currentQueue.toArray()));
    }

    public boolean isFull() {
        for (double current : currentQueue) {
            if (current < CURRENT_CUTOFF_FULL || Double.isNaN(current)) {
                return false;
            }
        }

        return true;
    }

    public boolean isEmpty() {
        if (power == 0) return false;

        for (double current : currentQueue) {
            if (current > CURRENT_CUTOFF_EMPTY || Double.isNaN(current)) {
                return false;
            }
        }

        return true;
    }

    public void spin(double power) {
        this.power = power;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public double current() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }
}
