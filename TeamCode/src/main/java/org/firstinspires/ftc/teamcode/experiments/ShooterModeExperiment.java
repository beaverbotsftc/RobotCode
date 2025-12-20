package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.router.Router;
import org.beaverbots.beaver.command.premade.router.Selector;
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
