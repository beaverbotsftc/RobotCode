package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndTransfer;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@TeleOp
public class ADeadSimpleTeleOp extends CommandOpMode {
    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private IntakeAndTransfer intake;
    private Turret turret;

    private GamepadEx gamepad;

    public void onInit() {
        voltageSensor = new VoltageSensor();
        drivetrain = new SwerveDrivetrain(voltageSensor);
        intake = new IntakeAndTransfer();
        turret = new Turret(voltageSensor);

        gamepad = new GamepadEx(gamepad1);

        register(voltageSensor, drivetrain, intake, turret, gamepad);
    }

    public void onStart() {
        schedule(
                new Repeat(() -> drivetrain.move(new Transform(gamepad.getLeftY(), -gamepad.getLeftX(), -gamepad.getRightX()))),
                new Repeat(() -> intake.intake(gamepad.getRightTrigger() - gamepad.getLeftTrigger()))
        );
    }

    public void periodic() {
    }
}
