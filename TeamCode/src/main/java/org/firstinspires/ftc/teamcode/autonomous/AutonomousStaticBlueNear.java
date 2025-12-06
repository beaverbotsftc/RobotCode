package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Parallel;
import org.beaverbots.BeaverCommand.util.RunUntil;
import org.beaverbots.BeaverCommand.util.Sequential;
import org.beaverbots.BeaverCommand.util.Stopwatch;
import org.beaverbots.BeaverCommand.util.Wait;
import org.beaverbots.BeaverCommand.util.WaitUntil;
import org.beaverbots.beavertracking.HolonomicFollowPath;
import org.beaverbots.beavertracking.PIDF;
import org.beaverbots.beavertracking.PIDFAxis;
import org.beaverbots.beavertracking.Path;
import org.beaverbots.beavertracking.PathBuilder;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousStaticBlueNear extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Stopper stopper;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private Localizer fusedLocalizer;

    private final Side side = Side.BLUE;
    private Motif motif;

    private boolean motifScanning = false;

    private Stopwatch stopwatch = new Stopwatch();

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        intake = new Intake();
        shooter = new Shooter();
        stopper = new Stopper();
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        limelight = new Limelight();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new DrivetrainState(0, 0, 0));

        register(gamepad, drivetrain, intake, shooter, stopper, pinpoint, limelight, fusedLocalizer);
        limelight.goalPipeline();
    }

    @Override
    public void periodicInit() {
        if (motifScanning) {
            Motif result = limelight.getMotif(side);
            if (result != null) motif = result;
            telemetry.addData("Limelight now:", result);
            telemetry.addData("Motif:", motif);
            telemetry.addLine(limelight.getStatus().toString());
        } else if (gamepad.getDpadUpJustPressed()) {
            pinpoint.setPosition(fusedLocalizer.getPosition());
            unregister(fusedLocalizer);
            limelight.obeliskPipeline();
            motifScanning = true;
        }

        if (!motifScanning) {
            telemetry.addData("Position:", fusedLocalizer.getPosition().toString());
        }
    }

    @Override
    public void onStart() {
        Pair<Path, Path> autoPart1 = new PathBuilder(List.of(pinpoint.getPosition().getX(), 144 - pinpoint.getPosition().getY(), -pinpoint.getPosition().getTheta()))
                .linearTo(new DrivetrainState(83.4, 58.7, -0.68).toList(), 0.3, 0.5)
                .stop(0.3, 0.5)
                .transform(List.of(x -> x, y -> 144 - y, theta -> 0.1 - theta))
                .build();

        Pair<Path, Path> autoPart2 = new PathBuilder(List.of(autoPart1.second.position(0).get(0), 144 - autoPart1.second.position(0).get(1), -autoPart1.second.position(0).get(2)))
                .linearTo(new DrivetrainState(86, 52, -Math.PI / 2).toList(), 0.3, 0.7)
                .stop(1, 1)
                .linearTo(new DrivetrainState(86, 28, -Math.PI / 2).toList(), 1, 4)
                .stop(0.2, 0.5)
                .transform(List.of(x -> x, y -> 144 - y, theta -> -theta))
                .build();

        Pair<Path, Path> autoPart3 = new PathBuilder(List.of(autoPart2.second.position(0).get(0), 144 - autoPart2.second.position(0).get(1), -autoPart2.second.position(0).get(2)))
                .linearTo(new DrivetrainState(83.4, 58.7, -0.68).toList(), 0.7, 0.7)
                .stop(0.3, 0.5)
                .transform(List.of(x -> x, y -> 144 - y, theta -> 0.1 - theta))
                .build();

        Pair<Path, Path> autoPart4 = new PathBuilder(List.of(autoPart3.second.position(0).get(0), 144 - autoPart3.second.position(0).get(1), -autoPart3.second.position(0).get(2)))
                .linearTo(new DrivetrainState(60, 52, -Math.PI / 2).toList(), 0.3, 0.7)
                .stop(0.5, 1)
                .linearTo(new DrivetrainState(60, 28, -Math.PI / 2).toList(), 1, 4)
                .stop(0.2, 0.5)
                .transform(List.of(x -> x, y -> 144 - y, theta -> -theta))
                .build();

        Pair<Path, Path> autoPart5 = new PathBuilder(List.of(autoPart4.second.position(0).get(0), 144 - autoPart4.second.position(0).get(1), -autoPart4.second.position(0).get(2)))
                .linearTo(new DrivetrainState(83.4, 58.7, -0.68).toList(), 1, 1)
                .stop(0.4, 0.8)
                .transform(List.of(x -> x, y -> 144 - y, theta -> 0.1 - theta))
                .build();

        Pair<Path, Path> autoPart6 = new PathBuilder(List.of(autoPart1.second.position(0).get(0), 144 - autoPart1.second.position(0).get(1), -autoPart1.second.position(0).get(2)))
                .linearTo(new DrivetrainState(120, 60, -0.68).toList(), 0.2, 0.5)
                .transform(List.of(x -> x, y -> 144 - y, theta -> -theta))
                .build();

        schedule(new Sequential(
                followPathTemplate(autoPart1.first),
                new Sequential(
                        new RunUntil(
                                new Sequential(
                                        new Instant(() -> shooter.spin(2150)),
                                        new WaitUntil(() -> Math.abs(shooter.getVelocity() - 2200) < 30),
                                        new Parallel(
                                                new Instant(() -> intake.spin(1)),
                                                new Instant(() -> stopper.spinForward())
                                        ),
                                        new Wait(2),
                                        new Instant(() -> shooter.spin(0))
                                ),
                                followPathTemplate(autoPart1.second)
                        ),
                        new Instant(() -> intake.spin(1)),
                        new Instant(() -> stopper.spinReverse()),
                        followPathTemplate(autoPart2.first),
                        new Instant(() -> intake.spin(0)),
                        new Instant(() -> stopper.spin(0)),
                        followPathTemplate(autoPart3.first),
                        new RunUntil(
                                new Sequential(
                                        new Instant(() -> shooter.spin(2150)),
                                        new WaitUntil(() -> Math.abs(shooter.getVelocity() - 2200) < 30),
                                        new Parallel(
                                                new Instant(() -> intake.spin(1)),
                                                new Instant(() -> stopper.spinForward())
                                        ),
                                        new Wait(2),
                                        new Instant(() -> shooter.spin(0))
                                ),
                                followPathTemplate(autoPart3.second)
                        ),
                        new Instant(() -> intake.spin(1)),
                        new Instant(() -> stopper.spinReverse()),
                        followPathTemplate(autoPart4.first),
                        new Instant(() -> intake.spin(0)),
                        new Instant(() -> stopper.spin(0)),
                        followPathTemplate(autoPart5.first),
                        new RunUntil(
                                new Sequential(
                                        new Instant(() -> shooter.spin(2150)),
                                        new WaitUntil(() -> Math.abs(shooter.getVelocity() - 2200) < 30),
                                        new Parallel(
                                                new Instant(() -> intake.spin(1)),
                                                new Instant(() -> stopper.spinForward())
                                        ),
                                        new Wait(2),
                                        new Instant(() -> shooter.spin(0))
                                ),
                                followPathTemplate(autoPart5.second)
                        ),
                        followPathTemplate(autoPart6.first),
                        new Instant(() -> requestOpModeStop())
                )));
        stopwatch.reset();
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
