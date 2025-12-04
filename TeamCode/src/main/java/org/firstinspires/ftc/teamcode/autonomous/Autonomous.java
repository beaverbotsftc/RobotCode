package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Parallel;
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
public class Autonomous extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Stopper stopper;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private Localizer fusedLocalizer;

    private final Side side = Side.RED;
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
        Pair<Path, Command> auto = new PathBuilder(fusedLocalizer.getPositionAsList())
                .linearToEase(new DrivetrainState(83.4, 58.7, -0.68).toList(), 6, 10)
                .linearToEase(new DrivetrainState(83.4, 58.7, -0.68).toList(), 2, 2)
                .addCommand(new Sequential(
                        new Instant(() -> shooter.spin(2200)),
                        new WaitUntil(() -> Math.abs(shooter.getVelocity() - 2200) < 30),
                        new Parallel(
                                new Instant(() -> intake.spin(1)),
                                new Instant(() -> stopper.spinForward())
                        ),
                        new Wait(5),
                        new Parallel(
                                new Instant(() -> intake.spin(0)),
                                new Instant(() -> stopper.spin(0)),
                                new Instant(() -> shooter.spin(0))
                        )
                ))
                .waitFor(10)
                .moveTo(new DrivetrainState(32, 36, 0).toList(), 5)
                .waitFor(5)
                .build(stopwatch);
        Command followPathCommand =
                new Parallel(
                        new HolonomicFollowPath(
                                auto.first,
                                new PIDF(List.of(
                                        new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, Constants.pidTauX)),
                                        new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, Constants.pidTauY)),
                                        new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, Constants.pidTauTheta)))),
                                pinpoint, drivetrain),
                        auto.second);
        schedule(new Sequential(
                followPathCommand,
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0)))
        ));
        stopwatch.reset();
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", pinpoint.getPosition().toString());
    }
}
