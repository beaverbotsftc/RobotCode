package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.List;

import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Sequential;
import org.beaverbots.beavertracking.HolonomicFollowPath;
import org.beaverbots.beavertracking.PIDF;
import org.beaverbots.beavertracking.PIDFAxis;
import org.beaverbots.beavertracking.Path;
import org.beaverbots.beavertracking.PathAxis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@Autonomous(group = "Experiments")
public class CircleAutoExperiment extends CommandRuntimeOpMode {
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;

    @Override
    public void onInit() {
        drivetrain = new MecanumDrivetrain(1);
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
    }

    @Override
    public void onStart() {
        register(drivetrain, pinpoint);

        schedule(
                new Sequential(
                        new HolonomicFollowPath(
                                new Path(
                                        List.of(
                                                new PathAxis(t -> 24 * Math.cos(t / 3) - 24, 0, 6 * Math.PI),
                                                new PathAxis(t -> 24 * Math.sin(t / 3), 0, 6 * Math.PI),
                                                new PathAxis(t -> -t, 0, 6 * Math.PI)),
                                        t -> t > 6 * Math.PI),
                                new PIDF(
                                        List.of(
                                                new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, 0.1)),
                                                new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, 0.1)),
                                                new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, 0.1)))),
                                pinpoint,
                                drivetrain),
                        new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0)))));
    }

    @Override
    public void periodic() {
    }
}
