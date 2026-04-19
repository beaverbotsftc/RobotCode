package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndTransfer;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@Autonomous
public class IntakeExperiment extends CommandOpMode {
    private VoltageSensor voltageSensor;
    private IntakeAndTransfer intakeAndTransfer;
    private GamepadEx gamepad;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        intakeAndTransfer = new IntakeAndTransfer();
        gamepad = new GamepadEx(gamepad1);

        register(voltageSensor, intakeAndTransfer, gamepad);
    }

    @Override
    public void periodic() {
        intakeAndTransfer.intake(gamepad.getRightTrigger() - gamepad.getLeftTrigger());
    }
}
