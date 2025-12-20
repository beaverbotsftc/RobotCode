package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.pidf.PIDF;
import org.beaverbots.beaver.pathing.pidf.PIDFAxis;
import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.PathAxis;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

import java.util.List;

@Autonomous(group = "Experiments")
public class ReturningExperiment extends CommandRuntimeOpMode {
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;

    @Override
    public void onInit() {
        drivetrain = new MecanumDrivetrain(0.3);
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
    }

    @Override
    public void onStart() {
        register(drivetrain, pinpoint);
        Path path = new Path(List.of(
                new PathAxis(t -> 0, 0, Double.POSITIVE_INFINITY),
                new PathAxis(t -> 0, 0, Double.POSITIVE_INFINITY),
                new PathAxis(t -> 0, 0, Double.POSITIVE_INFINITY)
        ), t -> false);
        PIDF pidf = new PIDF(List.of(
                new PIDFAxis(new PIDFAxis.K(0.1, 0.1, 0, 0, 0.4, 1, 0, 0)),
                new PIDFAxis(new PIDFAxis.K(0.1, 0.1, 0, 0, 0.4, 1, 0, 0)),
                new PIDFAxis(new PIDFAxis.K(1, 1, 0, 0, 0.4, 1, 0, 0))
        ));
        schedule(new HolonomicFollowPath(path, pidf, pinpoint, drivetrain));
    }

    @Override
    public void periodic() {
        telemetry.addData("Position", pinpoint.getPosition());
    }
}
