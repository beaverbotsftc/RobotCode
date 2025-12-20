package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Parallel;
import org.beaverbots.beaver.command.premade.RunUntil;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.command.premade.Wait;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.PIDF;
import org.beaverbots.beaver.pathing.PIDFAxis;
import org.beaverbots.beaver.pathing.Path;
import org.beaverbots.beaver.pathing.PathBuilder;
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
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousStaticRedFarPreloadLeave extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
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
        voltageSensor = new VoltageSensor();
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
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
        }

        if (!motifScanning) {
            telemetry.addData("Position:", fusedLocalizer.getPosition().toString());
        }
    }

    @Override
    public void onStart() {
        Pair<Path, Path> autoPart1 = new PathBuilder(List.of(pinpoint.getPositionAsList().get(0), pinpoint.getPositionAsList().get(1), pinpoint.getPositionAsList().get(2)))
                .linearTo(new DrivetrainState(18, 144 - 84, -Constants.shooterBias - 0.45).toList(), 0.3, 0.5)
                .stop(0.3, 0.5)
                .build();

        Pair<Path, Path> autoPart6 = new PathBuilder(List.of(autoPart1.second.position(0).get(0), autoPart1.second.position(0).get(1), autoPart1.second.position(0).get(2)))
                .linearTo(new DrivetrainState(10, 144 - 108, 0).toList(), 0.2, 0.5)
                .build();

        schedule(new Sequential(
                followPathTemplate(autoPart1.first),
                new Wait(10),
                new Sequential(
                        new RunUntil(
                                new Sequential(
                                        new Instant(() -> shooter.spin(3000)),
                                        new WaitUntil(() -> Math.abs(shooter.getVelocity() - 3000) < 30),
                                        new Parallel(
                                                new Instant(() -> intake.spin(1)),
                                                new Instant(() -> stopper.spinForward())
                                        ),
                                        new Wait(2),
                                        new Instant(() -> shooter.spin(0))
                                ),
                                followPathTemplate(autoPart1.second)
                        ),
                        new Wait(5),
                        followPathTemplate(autoPart6.first),
                        followPathTemplate(autoPart6.second)
                )));
        stopwatch.reset();
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", pinpoint.getPosition().toString());
        CrossModeStorage.position = pinpoint.getPosition();
    }


    public void onStop() {
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
