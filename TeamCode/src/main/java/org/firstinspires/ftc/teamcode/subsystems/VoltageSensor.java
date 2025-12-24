package org.firstinspires.ftc.teamcode.subsystems;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public class VoltageSensor implements Subsystem {
    private final com.qualcomm.robotcore.hardware.VoltageSensor sensor1;
    private final com.qualcomm.robotcore.hardware.VoltageSensor sensor2;

    private double voltage;

    public VoltageSensor() {
        sensor1 = HardwareManager.claim(com.qualcomm.robotcore.hardware.VoltageSensor.class, "Control Hub");
        sensor2 = HardwareManager.claim(com.qualcomm.robotcore.hardware.VoltageSensor.class, "Expansion Hub 2");
    }

    public void periodic() {
        voltage = Math.max(Math.min(sensor1.getVoltage(), sensor2.getVoltage()), 8); // Just to prevent division by zero errors
    }

    public double getVoltage() {
        return voltage;
    }
}
