package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.BeaverCommand.Command;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

public class IntakeControl implements Command {
    private Intake intake;
    private Stopper stopper;
    private ColorSensor colorSensor;
    private Gamepad gamepad;

    public IntakeControl(Intake intake, Stopper stopper, ColorSensor colorSensor, Gamepad gamepad) {
        this.intake = intake;
        this.stopper = stopper;
        this.colorSensor = colorSensor;
        this.gamepad = gamepad;
    }

    public boolean periodic() {
        double intakeSpeed = gamepad.getRightTrigger() - gamepad.getLeftTrigger();

        if(intakeSpeed > 0 && colorSensor.checkBack()){
            intake.setMaxPower(0.8);
            stopper.spinReverse();
        }else{
            intake.setMaxPower(1.0);
        }

        intake.spin(intakeSpeed);

        return false;
    }
}
