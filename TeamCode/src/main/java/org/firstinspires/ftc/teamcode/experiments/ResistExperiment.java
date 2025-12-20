package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.PIDF;
import org.beaverbots.beaver.pathing.PIDFAxis;
import org.beaverbots.beaver.pathing.Path;
import org.beaverbots.beaver.pathing.PathAxis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

import java.util.List;

@Autonomous(group = "Experiments")
public class ResistExperiment extends CommandRuntimeOpMode {
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer localizer;

    @Override
    public void onInit() {
        drivetrain = new MecanumDrivetrain(1);
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        limelight = new Limelight();
        limelight.goalPipeline();
        localizer = new FusedLocalizer(pinpoint, limelight, new DrivetrainState(0, 0, 0));
    }

    @Override
    public void onStart() {
        register(drivetrain, pinpoint, limelight, localizer);

        schedule(
                new HolonomicFollowPath(
                        new Path(
                                List.of(
                                        new PathAxis(t -> 0, 0, Double.POSITIVE_INFINITY),
                                        new PathAxis(t -> 0, 0, Double.POSITIVE_INFINITY),
                                        new PathAxis(t -> 0, 0, Double.POSITIVE_INFINITY)),
                                t -> false),
                        new PIDF(
                                List.of(
                                        new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, 0.1, Constants.pidGammaX)),
                                        new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, 0.1, Constants.pidGammaY)),
                                        new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, 0.1, Constants.pidGammaTheta)))),
                        localizer,
                        drivetrain)
        );
    }
}
