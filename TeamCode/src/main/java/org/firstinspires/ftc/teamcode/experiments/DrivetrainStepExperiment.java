package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.premade.Cycle;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDriveControl;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.function.DoubleUnaryOperator;

@Autonomous(group = "Experiments")
public class DrivetrainStepExperiment extends CommandOpMode {
    private Pinpoint pinpoint;
    private VoltageSensor voltageSensor;
    private SwerveDrivetrain drivetrain;

    private GamepadEx gamepad;

    @Override
    public void onInit() {
        pinpoint = new Pinpoint(new Transform(0, 0, 0));
        voltageSensor = new VoltageSensor();
        drivetrain = new SwerveDrivetrain(voltageSensor);

        gamepad = new GamepadEx(gamepad1);
    }

    double step = 0;

    @Override
    public void onStart() {
        register(pinpoint, voltageSensor, drivetrain, gamepad);

        schedule(
                new Repeat(() -> addData("Theta", Math.toDegrees(step) % 360)),
                new Cycle(i -> false,
                        new WaitUntil(() -> gamepad.getAJustPressed()),
                        new Instant(() -> step += Math.toRadians(20)),
                        new Instant(() -> drivetrain.move(new Transform(1, 0).rotateLateral(step)))
                ));
    }

    @Override
    public void periodic() {
        addData("X Position", pinpoint.getPosition().getX());
        addData("Y Position", pinpoint.getPosition().getY());
        addData("Theta Position", pinpoint.getPosition().getTheta() * 180 / Math.PI);
    }
}
