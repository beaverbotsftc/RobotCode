package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.PIDF;
import org.beaverbots.beaver.pathing.PIDFAxis;
import org.beaverbots.beaver.pathing.Path;
import org.beaverbots.beaver.pathing.PathAxis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

import java.util.List;

@Autonomous(group = "Experiments")
public class PathFollowingExperiment extends CommandRuntimeOpMode {
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;

    @Override
    public void onInit() {
        drivetrain = new MecanumDrivetrain(0.7);
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
    }

    @Override
    public void onStart() {
        register(drivetrain, pinpoint);
        Path path = new Path(List.of(
                new PathAxis(t -> 12 * Math.cos(t / 5), 0, Double.POSITIVE_INFINITY),
                new PathAxis(t -> 12 * Math.sin(t / 5), 0, Double.POSITIVE_INFINITY),
                new PathAxis(t -> t / 3, 0, Double.POSITIVE_INFINITY)
        ), t -> false);
        PIDF pidf = new PIDF(
                List.of(
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, Constants.pidTauX, Constants.pidGammaX)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, Constants.pidTauY, Constants.pidGammaY)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, Constants.pidTauTheta, Constants.pidGammaTheta))
                )
        );

        schedule(new HolonomicFollowPath(path, pidf, pinpoint, drivetrain));
    }

    @Override
    public void periodic() {
        telemetry.addData("Position", pinpoint.getPosition());
        telemetry.addData("Velocity", pinpoint.getVelocity());
    }
}
