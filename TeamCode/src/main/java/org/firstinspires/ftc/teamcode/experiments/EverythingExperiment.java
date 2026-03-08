package org.firstinspires.ftc.teamcode.experiments;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class EverythingExperiment extends CommandRuntimeOpMode {
    private Intake intake;

    private Gamepad gamepad;

    public void onInit() {
        intake = new Intake();

        gamepad = new Gamepad(gamepad1);
    }

    public void periodic() {
        if (gamepad.getTriangle()) {
            intake.intake();
        } else if (gamepad.getCross()) {
            intake.outtake();
        } else {
            intake.stop();
        }
    }
}
