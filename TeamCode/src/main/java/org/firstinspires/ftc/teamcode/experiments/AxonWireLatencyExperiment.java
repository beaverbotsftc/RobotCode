package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;

@Autonomous
public class AxonWireLatencyExperiment extends CommandRuntimeOpMode {
    CRServo servo;
    AnalogInput encoder;

    GamepadEx gamepad;

    Stopwatch stopwatch;
    double previousPosition = 0;
    double latency = 0;

    public void onInit() {
        servo = HardwareManager.claim(CRServo.class, "servo");
        encoder = HardwareManager.claim(AnalogInput.class, "encoder");

        servo.setDirection(DcMotorSimple.Direction.REVERSE);

        gamepad = new GamepadEx(gamepad1);

        register(gamepad);
    }

    public void periodic() {
        if (gamepad.getAJustPressed()) {
            schedule(new Sequential(new Instant(() -> stopwatch = new Stopwatch()),
                    new Instant(() -> previousPosition = encoder.getVoltage()),
                    new Instant(() -> servo.setPower(1)),
                    new WaitUntil(() -> Math.abs(encoder.getVoltage() - previousPosition) > 0.003),
                    new Instant(() -> latency = stopwatch.getElapsed()),
                    new Instant(() -> servo.setPower(0))
            ));
        }
        telemetry.addData("Latency", latency);
        telemetry.addData("Current voltage", encoder.getVoltage());
        telemetry.addData("Loop time", stopwatch == null ? 99999 : stopwatch.getDt());
    }
}