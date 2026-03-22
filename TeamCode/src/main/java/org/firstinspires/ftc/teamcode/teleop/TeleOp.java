package org.firstinspires.ftc.teamcode.teleop;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

public class TeleOp extends CommandRuntimeOpMode {
    private Drivetrain drivetrain;

    private Gamepad gamepad;

    public void onInit() {
        drivetrain = new MecanumDrivetrain();

        gamepad = new Gamepad(gamepad1);

        register(drivetrain, gamepad);
    }

    public void onStart() {
        schedule(new Repeat(() -> {
            drivetrain.move(new Transform(gamepad.getLeftY() * 12, gamepad.getLeftX() * 12, gamepad.getRightX()));
        }));
    }
}
