package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.beaver.command.Command;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

public class IntakeControl implements Command {
    private Intake intake;
    private Stopper stopper;
    private ColorSensor colorSensor;
    private Gamepad gamepad;
    private Led led;

    public IntakeControl(Intake intake, Stopper stopper, ColorSensor colorSensor, Led led, Gamepad gamepad) {
        this.intake = intake;
        this.stopper = stopper;
        this.colorSensor = colorSensor;
        this.gamepad = gamepad;
        this.led = led;
    }

    public boolean periodic() {
        double intakeSpeed = gamepad.getRightTrigger() - gamepad.getLeftTrigger();

        if(gamepad.getCircle()){
            stopper.spinForward();
            intake.setMaxPower(1);
            intakeSpeed = 1.0;
        }else if(intakeSpeed != 0){
            intake.setMaxPower(0.8);
            stopper.spinReverse();
        }else{
            stopper.stop();
        }

        if(colorSensor.checkBack() && colorSensor.checkFront()){
            led.setPurple();
        }else{
            led.turnOff();
        }

        intake.spin(intakeSpeed);

        return false;
    }
}
