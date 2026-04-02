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
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;

import java.util.List;

@Autonomous
public class InfiniteServoExperiment extends CommandRuntimeOpMode {
    CRServo servo;
    AnalogInput encoder;

    PIDFAxis pidf = new PIDFAxis(new PIDFAxis.K(
            //0.8 * 0.333, 0.666 * 0.8 / 0.25, 0.25 * 0.8 * 0.111, 0, 1, 1, 0.5, 100
            //0.8 * 0.2, 0.4 * 0.8 / 0.25, 0.25 * 0.8 * 0.0666, 0, 0, 1, 0.1, 0
            // Auto-tuned loss: 0.0416
            0.3239, 0.9773, 0.0001, new double[] {0}, 0, 1, 0.2072, 527.6937
    ));


    GamepadEx gamepad;

    Stopwatch stopwatch;

    InfiniteServo infiniteServo;

    public void onInit() {
        servo = HardwareManager.claim(CRServo.class, "servo");
        encoder = HardwareManager.claim(AnalogInput.class, "encoder");

        servo.setDirection(DcMotorSimple.Direction.REVERSE);

        gamepad = new GamepadEx(gamepad1);

        infiniteServo = new InfiniteServo(servo, encoder, pidf, 0);

        register(gamepad, infiniteServo);
    }

    public void onStart() {
        stopwatch = new Stopwatch();
    }

    public void periodic() {
        infiniteServo.setAngle(gamepad.getRightX() * 2 * Math.PI);
    }
}
