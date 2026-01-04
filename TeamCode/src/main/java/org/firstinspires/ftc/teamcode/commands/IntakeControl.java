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
    private Led led;
    private Gamepad gamepad;

    public IntakeControl(Intake intake, Stopper stopper, ColorSensor colorSensor, Led led, Gamepad gamepad) {
        this.intake = intake;
        this.stopper = stopper;
        this.colorSensor = colorSensor;
        this.led = led;
        this.gamepad = gamepad;
    }

    public boolean periodic() {
        double intakeSpeed = gamepad.getRightTrigger() - gamepad.getLeftTrigger();

        if(gamepad.getCircle()){ //balls go into shooter
            intake.setMaxPower(0.95);
            stopper.setMaxPower(0.95);
            if(ShooterControl.percentError < 7.0 && ShooterControl.percentError >= 0.0){
                intakeSpeed = 1.0;
                stopper.spinForward();
            }else{
                intakeSpeed = 0.0;
                stopper.stop();
            }
        }else if(intakeSpeed != 0){ //when intaking balls
            intake.setMaxPower(1);
            stopper.setMaxPower(0.5);

            stopper.spinReverse();
        }else{
            stopper.stop();
        }

        if(colorSensor.hasThreeBalls()){
            led.setBallLedPurple();
        }else{
            led.turnOffBallLed();
        }

        intake.spin(intakeSpeed);

        return false;
    }
}
