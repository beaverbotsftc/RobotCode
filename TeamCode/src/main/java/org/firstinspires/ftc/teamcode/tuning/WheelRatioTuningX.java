package org.firstinspires.ftc.teamcode.tuning;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Cycle;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.RunUntil;
import org.beaverbots.beaver.command.premade.Wait;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

public class WheelRatioTuningX extends CommandRuntimeOpMode {
    private Drivetrain drivetrain;
    private GamepadEx gamepad;

    public void onInit() {
        VoltageSensor voltageSensor = new VoltageSensor();
        drivetrain = new MecanumDrivetrain(voltageSensor);
        gamepad = new GamepadEx(gamepad1);

        register(voltageSensor, drivetrain, gamepad);
    }

    public void onStart() {
        schedule(new Cycle(
                i -> false,
                new WaitUntil(() -> gamepad.getX()),
                new Instant(() -> drivetrain.move(new Transform(1, 0, 0))),
                new Wait(2),
                new RunUntil(new WaitUntil(() -> gamepad.getX()), new Repeat(() -> {
                    telemetry.addData("left front", Constants.drivetrainFrontLeftFactor);
                    telemetry.addData("right front", Constants.drivetrainFrontRightFactor);
                    telemetry.addData("left back", Constants.drivetrainBackLeftFactor);
                    telemetry.addData("right back", Constants.drivetrainBackRightFactor);
                    Constants.drivetrainFrontLeftFactor += gamepad.getLeftX() * 0.001;
                    Constants.drivetrainFrontRightFactor += gamepad.getLeftY() * 0.001;
                    Constants.drivetrainBackLeftFactor += gamepad.getRightX() * 0.001;
                    Constants.drivetrainBackRightFactor += gamepad.getRightY() * 0.001;
                }))
        ));
    }
}
