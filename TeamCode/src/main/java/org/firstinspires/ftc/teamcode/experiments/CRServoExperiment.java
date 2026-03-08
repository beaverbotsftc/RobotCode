package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.beaver.InfiniteServo;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;

import java.util.List;

@Autonomous
public class CRServoExperiment extends CommandRuntimeOpMode {
    CRServo servo;

    Gamepad gamepad;

    public void onInit() {
        servo = HardwareManager.claim(CRServo.class, "servo");

        servo.setDirection(DcMotorSimple.Direction.REVERSE);

        gamepad = new Gamepad(gamepad1);

        register(gamepad);
    }

    public void periodic() {
        servo.setPower(gamepad.getRightX());
    }
}
