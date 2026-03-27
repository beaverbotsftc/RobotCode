package org.firstinspires.ftc.teamcode.experiments;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SwerveModule;

public class SwerveModuleExperiment extends CommandRuntimeOpMode {
    GamepadEx gamepad;
    SwerveModule module;

    public void onInit() {
        gamepad = new GamepadEx(gamepad1);
        module = new SwerveModule();

        register(gamepad, module);
    }

    public void periodic() {
        module.drive(new Transform(gamepad.getLeftX(), gamepad.getLeftY()));
    }
}
