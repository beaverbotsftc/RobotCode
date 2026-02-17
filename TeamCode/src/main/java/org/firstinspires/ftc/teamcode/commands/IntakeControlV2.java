package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.beaver.command.Command;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

public class IntakeControlV2 implements Command {
    private final Intake intake;
    private final Stopper stopper;
    private final Gamepad gamepad;

    public IntakeControlV2(Intake intake, Stopper stopper, Gamepad gamepad) {
        this.intake = intake;
        this.stopper = stopper;
        this.gamepad = gamepad;
    }


    public boolean periodic() {
        double intakeSpeed = gamepad.getRightTrigger() - gamepad.getLeftTrigger();

        intake.spin(intakeSpeed);
        if (intakeSpeed == 0) {
            stopper.spin(0);
        } else {
            stopper.spin(-0.8);
        }

        return false;
    }
}
