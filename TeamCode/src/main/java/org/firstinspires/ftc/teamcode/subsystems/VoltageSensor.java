package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public class VoltageSensor implements Subsystem {
    private com.qualcomm.robotcore.hardware.VoltageSensor sensor1;
    private com.qualcomm.robotcore.hardware.VoltageSensor sensor2;

    private final int readFrequency;
    private final int readOffset;

    private double voltage;

    public VoltageSensor(int readFrequency, int readOffset) {
        this.readFrequency = readFrequency;
        this.readOffset = readOffset;

        sensor1 = HardwareManager.get(com.qualcomm.robotcore.hardware.VoltageSensor.class, "Control Hub");
        try {
            sensor2 = HardwareManager.get(com.qualcomm.robotcore.hardware.VoltageSensor.class, "Expansion Hub 2");
        } catch (RuntimeException e) {
            sensor2 = null;
            RobotLog.ee("VoltageSensor", "Expansion Hub not found.");
        }
    }

    public VoltageSensor() {
        this(1, 0);
    }

    public void periodic() {
        if (CommandOpMode.loopNumber == 0 || (CommandOpMode.loopNumber + readOffset) % readFrequency == 0) {
            // Sensor 2 is unnecessary
            //if (sensor2 != null)
            //    voltage = Math.min(sensor1.getVoltage(), sensor2.getVoltage());
            //else
                voltage = sensor1.getVoltage();
        }
    }

    public double getVoltage() {
        return Math.max(voltage, 5); // Just to prevent division by zero errors, should *never* spike below 7, let alone 5.
    }
}
