package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDriveControl;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.function.DoubleUnaryOperator;

@Autonomous(group = "Experiments")
public class DrivetrainExperiment extends CommandOpMode {
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

    @Override
    public void onStart() {
        register(pinpoint, voltageSensor, drivetrain, gamepad);

        schedule(new SwerveDriveControl(drivetrain, pinpoint, gamepad, new DoubleUnaryOperator[] {x -> x, y -> y, theta -> theta}, theta -> theta));
    }

    @Override
    public void periodic() {
        addData("X Position", pinpoint.getPosition().getX());
        addData("Y Position", pinpoint.getPosition().getY());
        addData("Theta Position", pinpoint.getPosition().getTheta() * 180 / Math.PI);
    }
}
