package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.RunUntil;
import org.beaverbots.BeaverCommand.util.Sequential;
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
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousRedNearStaticV2 extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Stopper stopper;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer fusedLocalizer;

    private final Side side = Side.RED;
    private Motif motif;

    private boolean motifScanning = false;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        drivetrain.toggleBrake();
        intake = new Intake();
        shooter = new Shooter(voltageSensor);
        stopper = new Stopper();
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        limelight = new Limelight();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new DrivetrainState(0, 0, 0));

        register(voltageSensor, gamepad, drivetrain, intake, shooter, stopper, pinpoint, limelight, fusedLocalizer);
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
            intake.spin(0);
            stopper.spin(0);
        }

        if (!motifScanning) {
            telemetry.addData("Position:", fusedLocalizer.getPosition().toString());
            telemetry.addData("Covariance:", fusedLocalizer.getCovariance().getData()[0][0]);
            telemetry.addData("Covariance:", fusedLocalizer.getCovariance().getData()[1][1]);
            telemetry.addData("Covariance:", fusedLocalizer.getCovariance().getData()[2][2]);
        }
    }

    @Override
    public void onStart() {
        Pair<Path, Path> autoPart1 = new PathBuilder(pinpoint.getPositionAsList())
                .linearTo(new DrivetrainState(93.67, 51.38, -0.69).toList(), 0.3, 1)
                .stop(0.3, 0.5)
                .build();

        Pair<Path, Path> autoPart2 = new PathBuilder(autoPart1.second)
                .linearTo(new DrivetrainState(83.85, 55.95, -Math.PI / 2).toList(), 0.3, 1)
                .stop(0.3, 0.5)
                .linearTo(new DrivetrainState(83.85, 25, -Math.PI / 2).toList(), 0.3, 3)
                .stop(0.3, 0.5)
                .build();

        Pair<Path, Path> autoPart3 = new PathBuilder(autoPart2.second)
                .linearTo(new DrivetrainState(93.67, 51.38, -0.69).toList(), 0.3, 1)
                .stop(0.3, 0.5)
                .build();

        Pair<Path, Path> autoPart4 = new PathBuilder(autoPart3.second)
                .linearTo(new DrivetrainState(59.85, 55.95, -Math.PI / 2).toList(), 0.3, 1)
                .stop(0.3, 0.5)
                .linearTo(new DrivetrainState(59.85, 25, -Math.PI / 2).toList(), 0.3, 3)
                .stop(0.3, 0.5)
                .build();

        Pair<Path, Path> autoPart5 = new PathBuilder(autoPart4.second)
                .linearTo(new DrivetrainState(93.67, 51.38, -0.69).toList(), 0.3, 1)
                .stop(0.3, 0.5)
                .build();

        Pair<Path, Path> autoPart6 = new PathBuilder(autoPart5.second)
                .linearTo(new DrivetrainState(120, 51.38, -0.69).toList(), 0.3, 1)
                .stop(0.3, 0.5)
                .build();

        schedule(
                new Sequential(
                        new Instant(() -> shooter.spin(2300)),
                        new Instant(() -> shooter.setHood(0.27)),
                        followPathTemplate(autoPart1.first),
                        new RunUntil(
                                new Sequential(
                                        new WaitUntil(() -> Math.abs(shooter.getVelocity() - 2300) < 50),
                                        new Instant(() -> intake.spin(1)),
                                        new Instant(() -> stopper.spinForward()),
                                        new Wait(2)
                                ),
                                followPathTemplate(autoPart1.second)
                        ),
                        new Instant(() -> stopper.spinReverse()),
                        followPathTemplate(autoPart2.first),
                        new Instant(() -> intake.spin(0)),
                        new Instant(() -> stopper.spin(0)),
                        followPathTemplate(autoPart3.first),
                        new RunUntil(
                                new Sequential(
                                        new WaitUntil(() -> Math.abs(shooter.getVelocity() - 2300) < 50),
                                        new Instant(() -> intake.spin(1)),
                                        new Instant(() -> stopper.spinForward()),
                                        new Wait(2)
                                ),
                                followPathTemplate(autoPart3.second)
                        ),
                        new Instant(() -> stopper.spinReverse()),
                        followPathTemplate(autoPart4.first),
                        new Instant(() -> intake.spin(0)),
                        new Instant(() -> stopper.spin(0)),
                        followPathTemplate(autoPart5.first),
                        new RunUntil(
                                new Sequential(
                                        new WaitUntil(() -> Math.abs(shooter.getVelocity() - 2300) < 50),
                                        new Instant(() -> intake.spin(1)),
                                        new Instant(() -> stopper.spinForward()),
                                        new Wait(2)
                                ),
                                followPathTemplate(autoPart5.second)
                        ),
                        new Instant(() -> shooter.spin(0)),
                        new Instant(() -> intake.spin(0)),
                        new Instant(() -> stopper.spin(0)),
                        followPathTemplate(autoPart6.first),
                        followPathTemplate(autoPart6.second)
                )
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", pinpoint.getPosition().toString());
        CrossModeStorage.position = pinpoint.getPosition();
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
