package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;
import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Sequential;
import org.beaverbots.BeaverCommand.util.Stopwatch;
import org.beaverbots.beavertracking.HolonomicFollowPath;
import org.beaverbots.beavertracking.PIDF;
import org.beaverbots.beavertracking.PIDFAxis;
import org.beaverbots.beavertracking.Path;
import org.beaverbots.beavertracking.PathBuilder;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousDemo extends CommandRuntimeOpMode {
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;

    private Stopwatch stopwatch = new Stopwatch();

    public void onInit() {
        drivetrain = new MecanumDrivetrain();
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));

        register(drivetrain, pinpoint);
    }

    public void onStart() {
        Pair<Path, Path> autonomous = new PathBuilder(pinpoint.getPositionAsList())
                .bezierTo(
                        List.of(0.0, 0.0, 0.0),
                        List.of(17.7, 21.1, 0.0),
                        List.of(19.7, 8.6, 0.0),
                        0,
                        6
                )
                .bezierTo(
                        List.of(21.7, -3.900000000000002, 0.0),
                        List.of(15.94, 25.36, 0.0),
                        List.of(15.94, 25.36, 0.0),
                        0,
                        6
                )
                .build();
        schedule(
                new Sequential(
                        followPathTemplate(autonomous.first),
                        followPathTemplate(autonomous.second)
                )
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", pinpoint.getPosition().toString());
    }

    private Command followPathTemplate(Path path) {
        return new Sequential(
                new HolonomicFollowPath(
                        path,
                        new PIDF(List.of(
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, Constants.pidTauX, Constants.pidGammaX)),
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, Constants.pidTauY, Constants.pidGammaY)),
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, Constants.pidTauTheta, Constants.pidGammaTheta)))),
                        pinpoint, drivetrain),
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0))));
    }
}