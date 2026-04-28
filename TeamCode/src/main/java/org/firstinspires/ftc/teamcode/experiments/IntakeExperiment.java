package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@Autonomous
public class IntakeExperiment extends CommandOpMode {
    private VoltageSensor voltageSensor;
    private Intake intake;
    private GamepadEx gamepad;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        intake = new Intake();
        gamepad = new GamepadEx(gamepad1);

        register(voltageSensor, intake, gamepad);
    }

    @Override
    public void periodic() {
        intake.intake(gamepad.getRightTrigger() - gamepad.getLeftTrigger());
    }
}
