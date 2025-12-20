package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.List;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.pidf.PIDF;
import org.beaverbots.beaver.pathing.pidf.PIDFAxis;
import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.PathAxis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@Autonomous(group = "Experiments")
public class CircleAutoExperiment extends CommandRuntimeOpMode {
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer fusedLocalizer;

    @Override
    public void onInit() {
        drivetrain = new MecanumDrivetrain(1);
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        limelight = new Limelight();
        limelight.goalPipeline();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new DrivetrainState(0, 0, 0));

        register(drivetrain, pinpoint, fusedLocalizer, limelight);
    }

    @Override
    public void periodicInit() {
        DrivetrainState pos = fusedLocalizer.getPosition();
        telemetry.addData("Position:", pos);
        if (gamepad1.leftBumperWasPressed()) {
            pinpoint.setPosition(pos);
        }
    }

    @Override
    public void onStart() {
        schedule(
                new Sequential(
                        new HolonomicFollowPath(
                                new Path(
                                        List.of(
                                                new PathAxis(t -> 24 * Math.cos(t) - 24 + 72, 0, 6 * Math.PI),
                                                new PathAxis(t -> 24 * Math.sin(t) + 72, 0, 6 * Math.PI),
                                                new PathAxis(t -> -t, 0, 6 * Math.PI)),
                                        t -> t > 6 * Math.PI),
                                new PIDF(
                                        List.of(
                                                new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, Constants.pidTauX, Constants.pidGammaX)),
                                                new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, Constants.pidTauY, Constants.pidGammaY)),
                                                new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, Constants.pidTauTheta, Constants.pidGammaTheta))
                                        )
                                ),
                                pinpoint,
                                drivetrain),
                        new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0)))));
    }

    @Override
    public void periodic() {
    }
}
