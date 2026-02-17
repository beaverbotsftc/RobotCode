package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@Autonomous
public class IntakeSpeedExperiment extends CommandRuntimeOpMode {
    VoltageSensor voltageSensor;
    Shooter shooter;
    Stopper stopper;
    Intake intake;
    Gamepad gamepad;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        shooter = new Shooter(voltageSensor);
        stopper = new Stopper();
        intake = new Intake();
        gamepad = new Gamepad(gamepad1);

        register(shooter, stopper, intake, gamepad, voltageSensor);
    }

    @Override
    public void periodic() {
        shooter.setFlywheelSpeed(2200);
        shooter.setHoodAngle(0.4);
        stopper.spin(gamepad.getLeftX());
        intake.spin(gamepad.getLeftX());
    }
}
