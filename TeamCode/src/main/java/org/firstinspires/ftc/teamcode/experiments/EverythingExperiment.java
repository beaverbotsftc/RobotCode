package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@Autonomous
public class EverythingExperiment extends CommandRuntimeOpMode {
    private Intake intake;
    private Turret turret;
    private Drivetrain drivetrain;
    private VoltageSensor voltageSensor;

    private GamepadEx gamepad;

    public void onInit() {
        intake = new Intake();
        voltageSensor = new VoltageSensor();
        turret = new Turret(voltageSensor);
        drivetrain = new MecanumDrivetrain(voltageSensor);

        gamepad = new GamepadEx(gamepad1);

        register(voltageSensor, intake, turret, drivetrain, gamepad);
    }

    public void periodic() {
        turret.turn(2 * (gamepad.getRightTrigger() - gamepad.getLeftTrigger()) + Math.PI);

        if (gamepad.getY()) {
            intake.intake(true);
        } else if (gamepad.getA()) {
            intake.intake(false);
        } else {
            intake.stop();
        }
        intake.transfer(gamepad.getB());

        turret.shoot(5000 * gamepad.getRightY());

        drivetrain.move(new Transform(gamepad.getLeftY() / Constants.drivetrainPowerConversionFactorX, -gamepad.getLeftX() / Constants.drivetrainPowerConversionFactorY, gamepad.getRightX() / Constants.drivetrainPowerConversionFactorTheta));
    }
}
