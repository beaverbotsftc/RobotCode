package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.commands.SimpleControl;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@Disabled
public class Debug extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Pinpoint pinpoint;

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        intake = new Intake();
        shooter = new Shooter();
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));

    }

    @Override
    public void onStart() {
        register(gamepad, drivetrain, intake, shooter, pinpoint);
        schedule(new SimpleControl(gamepad, drivetrain, intake, shooter));
    }

    @Override
    public void periodic() {
        telemetry.addLine(pinpoint.getPosition().toString());
        telemetry.addLine(pinpoint.getVelocity().toString());
    }
}
