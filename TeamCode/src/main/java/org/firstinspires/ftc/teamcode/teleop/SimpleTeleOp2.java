package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;

@TeleOp
public class SimpleTeleOp2 extends CommandRuntimeOpMode {
    Servo s;

    @Override
    public void onInit() {
        s = hardwareMap.get(Servo.class, "hood");
    }

    double a = 0;

    @Override
    public void periodic() {
        a += gamepad1.left_stick_x / 1000;
        telemetry.addData("a",a);
        s.setPosition(a);
    }
}