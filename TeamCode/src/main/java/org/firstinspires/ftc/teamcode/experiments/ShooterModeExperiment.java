package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.router.Router;
import org.beaverbots.BeaverCommand.util.router.Selector;
import org.beaverbots.beavertracking.HolonomicFollowPath;
import org.beaverbots.beavertracking.PIDF;
import org.beaverbots.beavertracking.PIDFAxis;
import org.beaverbots.beavertracking.Path;
import org.beaverbots.beavertracking.PathAxis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.commands.DrivetrainControl;
import org.firstinspires.ftc.teamcode.commands.ShooterMode;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.List;

@Autonomous(group = "Experiments")
public class ShooterModeExperiment extends CommandRuntimeOpMode {
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer localizer;
    private Gamepad gamepad;

    @Override
    public void onInit() {
        drivetrain = new MecanumDrivetrain(1);
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        limelight = new Limelight();
        limelight.goalPipeline();
        localizer = new FusedLocalizer(pinpoint, limelight, new DrivetrainState(0, 0, 0));
        gamepad = new Gamepad(gamepad1);
    }

    @Override
    public void onStart() {
        register(drivetrain, pinpoint, limelight, localizer, gamepad);

        schedule(
                new Router(
                        new Selector(() -> gamepad1.left_stick_button),
                        new DrivetrainControl(drivetrain, gamepad),
                        new ShooterMode(localizer, drivetrain, Side.RED, false)
                )
        );
    }

    public void periodic() {
        telemetry.addData("Location: ", localizer.getPosition().toString());
    }
}
