package org.firstinspires.ftc.teamcode.commands;

import android.util.Pair;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.util.Geometry;
import org.beaverbots.beaver.util.PiecewiseLinearFunction;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.List;

public class IntakeControl implements Command {
    private Intake intake;
    private Stopper stopper;
    private ColorSensor colorSensor;
    private Led led;
    private Gamepad gamepad;
    private Localizer localizer;
    private boolean restrict;
    private Side side;

    public IntakeControl(Intake intake, Stopper stopper, Localizer localizer, boolean restrict, Side side, ColorSensor colorSensor, Led led, Gamepad gamepad) {
        this.intake = intake;
        this.stopper = stopper;
        this.colorSensor = colorSensor;
        this.led = led;
        this.gamepad = gamepad;
        this.localizer = localizer;
        this.restrict = restrict;
        this.side = side;
    }


    public boolean periodic() {
        double intakeSpeed = gamepad.getRightTrigger() - gamepad.getLeftTrigger();

        if (gamepad.getRightBumper()) {
            intake.spin(1.0);
            stopper.spinForward();
            intake.empty();
        } else {
            intake.spin(intakeSpeed);
            if (intakeSpeed == 0) {
                stopper.spin(0);
            } else {
                stopper.spin(-0.8);
            }

            if (intakeSpeed < 0) {
                intake.empty();
            }
        }

        return false;
    }
}
