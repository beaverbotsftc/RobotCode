package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndTransfer;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@TeleOp
public class TheTeleOpOfTheRobot extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private IntakeAndTransfer intake;
    private Turret turret;

    private Pinpoint pinpoint;

    private GamepadEx gamepad;

    public void onInit() {
        voltageSensor = new VoltageSensor();
        drivetrain = new SwerveDrivetrain(voltageSensor);
        intake = new IntakeAndTransfer();
        turret = new Turret(voltageSensor);

        pinpoint = new Pinpoint(new Transform(0, 0, 0));

        gamepad = new GamepadEx(gamepad1);

        register(voltageSensor, drivetrain, intake, turret, pinpoint, gamepad);
    }

    double rpm = 0;

    public void onStart() {
        schedule(
                new Repeat(() -> drivetrain.move(new Transform(gamepad.getLeftY(), -gamepad.getLeftX(), -gamepad.getRightX()).toLocalVelocity(pinpoint.getPosition()))),
                new Repeat(() -> intake.intake(gamepad.getRightTrigger() - gamepad.getLeftTrigger())),
                new Repeat(() -> intake.transfer(gamepad.getRightBumper()))
        );
    }

    public void periodic() {
        telemetry.addData("RPM", rpm);
        rpm += gamepad.getDpadUpJustPressed() ? 100 : 0;
        rpm -= gamepad.getDpadDownJustPressed() ? 100 : 0;
        turret.shoot(rpm);
    }
}
