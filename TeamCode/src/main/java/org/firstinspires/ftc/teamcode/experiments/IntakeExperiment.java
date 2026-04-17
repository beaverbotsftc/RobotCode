package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndTransfer;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@Autonomous
public class IntakeExperiment extends CommandRuntimeOpMode {
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
