package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AxonRunToPositionExperiment extends CommandRuntimeOpMode {
    CRServo servo;
    AnalogInput encoder;

    PIDF pidf = new PIDF(List.of(new PIDFAxis(new PIDFAxis.K(
            //0.8 * 0.333, 0.666 * 0.8 / 0.25, 0.25 * 0.8 * 0.111, 0, 1, 1, 0.5, 100
            //0.8 * 0.2, 0.4 * 0.8 / 0.25, 0.25 * 0.8 * 0.0666, 0, 0, 1, 0.1, 0
            // Auto-tuned loss: 0.0416
            0.3239, 0.9773, 0.0001, 0, 0, 1, 0.2072, 527.6937
    ))));

    double axonDelay = 0.00;

    double readWindow = 0.1;

    List<Double> times = new ArrayList<>(List.of(0., 0., 0.));
    List<Double> positions = new ArrayList<>(List.of(0., 0., 0.));

    Gamepad gamepad;

    Stopwatch stopwatch;

    public void onInit() {
        servo = HardwareManager.claim(CRServo.class, "servo");
        encoder = HardwareManager.claim(AnalogInput.class, "encoder");

        servo.setDirection(DcMotorSimple.Direction.REVERSE);

        gamepad = new Gamepad(gamepad1);

        register(gamepad);
    }

    public void onStart() {
        stopwatch = new Stopwatch();
    }

    public void periodic() {
        double angle = encoder.getVoltage() / 3.3 * 2.0 * Math.PI;
        double time = stopwatch.getElapsed();
        times.add(time - axonDelay);
        positions.add(angle);

        double actualAngle;
        int  i = 0;
        while (true) {
            if (times.get(i) >= time - axonDelay - readWindow) break;
            i++;
        }

        actualAngle = (angle - positions.get(i)) / (time - times.get(i) + 0.0001) * axonDelay + angle;

        telemetry.addData("Angle (deg)", actualAngle / Math.PI * 180.0);
        telemetry.addData("Angle (V)", encoder.getVoltage());
        telemetry.addData("Max Angle (V)", encoder.getMaxVoltage());

        double desiredAngle = (gamepad.getRightX() + 1) * Math.PI;

        double control = pidf.update(List.of(actualAngle - desiredAngle), List.of(desiredAngle), stopwatch.getDt()).get(0);
        servo.setPower(control);

        telemetry.addData("Desired angle (deg)", desiredAngle / Math.PI * 180.0);
        telemetry.addData("Power given", control);
    }
}
