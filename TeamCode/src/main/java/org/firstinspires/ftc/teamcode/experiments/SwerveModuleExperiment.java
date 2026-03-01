package org.firstinspires.ftc.teamcode.experiments;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.Transform;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SwerveModule;

public class SwerveModuleExperiment extends CommandRuntimeOpMode {
    Gamepad gamepad;
    SwerveModule module;

    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        module = new SwerveModule();

        register(gamepad, module);
    }

    public void periodic() {
        module.drive(new Transform(gamepad.getLeftX(), gamepad.getLeftY()));
    }
}
