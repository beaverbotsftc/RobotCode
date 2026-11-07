package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.beaver.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.beaver.command.HardwareManager;

@Autonomous
public class SwerveSteeringLatencyExperiment extends CommandOpMode {
    CRServo x;
    AnalogInput y;

    double e1;

    public void onInit() {
        x = HardwareManager.claim("front left swervo");
        y = HardwareManager.claim("front left swervo encoder");
    }

    public void onStart() {
        e1 = y.getVoltage();
    }

    public void periodic() {
        addData("y", y.getVoltage());
        if (gamepad.getAJustPressed()) {
            x.setPower(1);
            addData("y", 3.3);
        }
    }
}
