package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@Autonomous
public class EverythingExperiment extends CommandRuntimeOpMode {
    private Intake intake;
    private Turret turret;
    private Drivetrain drivetrain;

    private Gamepad gamepad;

    public void onInit() {
        intake = new Intake();
        turret = new Turret();
        drivetrain = new MecanumDrivetrain();

        gamepad = new Gamepad(gamepad1);

        register(intake, turret, drivetrain, gamepad);
    }

    public void periodic() {
        if (gamepad.getTriangle()) {
            intake.intake();
        } else if (gamepad.getCross()) {
            intake.outtake();
        } else {
            intake.stop();
        }

        //turret.turn((gamepad.getRightTrigger() - gamepad.getLeftTrigger()) * 1.5 + Math.PI);

        drivetrain.move(new Transform(0, gamepad.getRightTrigger() * 12, gamepad.getLeftTrigger() * Math.PI / 3));
    }
}
