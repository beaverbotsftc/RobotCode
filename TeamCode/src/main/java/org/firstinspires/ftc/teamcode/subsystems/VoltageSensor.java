package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public class VoltageSensor implements Subsystem {
    private com.qualcomm.robotcore.hardware.VoltageSensor sensor1;
    private com.qualcomm.robotcore.hardware.VoltageSensor sensor2;

    private double voltage;

    public VoltageSensor() {
        sensor1 = HardwareManager.claim(com.qualcomm.robotcore.hardware.VoltageSensor.class, "Control Hub");
        try {
            sensor2 = HardwareManager.claim(com.qualcomm.robotcore.hardware.VoltageSensor.class, "Expansion Hub 2");
        } catch (RuntimeException e) {
            sensor2 = null;
            RobotLog.ee("VoltageSensor", "Expansion Hub not found.");
        }
    }

    public void periodic() {
        if (sensor2 != null)
            voltage = Math.max(Math.min(sensor1.getVoltage(), sensor2.getVoltage()), 5); // Just to prevent division by zero errors, should *never* spike below 7, let alone 5.
        else
            voltage = sensor1.getVoltage();
    }

    public double getVoltage() {
        return voltage;
    }
}
