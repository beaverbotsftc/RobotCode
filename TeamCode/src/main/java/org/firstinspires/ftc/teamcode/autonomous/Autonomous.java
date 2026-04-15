package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.RunUntil;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.Wait;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.pathbuilder.PathBuilder;
import org.beaverbots.beaver.pathing.trackers.HolonomicPathTracker;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Geometry;
import org.beaverbots.beaver.util.Transform;
import org.beaverbots.beaver.util.Triple;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

import java.util.ArrayList;
import java.util.List;
import java.util.function.ToDoubleFunction;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends CommandRuntimeOpMode {
    private GamepadEx gamepad;
    private SwerveDrivetrain drivetrain;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer fusedLocalizer;
    private VoltageSensor voltageSensor;

    private Transform currentPosition;

    private List<Path> paths = new ArrayList<>();
    private List<Path> pathsHold = new ArrayList<>();

    private ToDoubleFunction<Triple<List<Double>, List<Double>, List<Double>>> usageRatio;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        gamepad = new GamepadEx(gamepad1);
        pinpoint = new Pinpoint(new Transform(0, 0, 0));
        limelight = new Limelight(Limelight.Pipeline.LOCALIZATION_GOAL);
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new Transform(0, 0, 0));
        drivetrain = new SwerveDrivetrain(voltageSensor);

        usageRatio = PathBuilder.createHolonomicUsage(1 / Constants.drivetrainPowerConversionFactorX, 1 / Constants.drivetrainPowerConversionFactorY, 1 / Constants.drivetrainPowerConversionFactorTheta, 9999, 9999, 9999);

        register(voltageSensor, gamepad, pinpoint, limelight, fusedLocalizer, drivetrain);
    }

    public void periodicInit() {
        telemetry.addData("Position:", fusedLocalizer.getPosition().toString());
        telemetry.addData("Variance X:", fusedLocalizer.getCovariance().getEntry(0, 0));
        telemetry.addData("Variance Y:", fusedLocalizer.getCovariance().getEntry(1, 1));
        telemetry.addData("Variance Theta:", fusedLocalizer.getCovariance().getEntry(2, 2));
    }

    @Override
    public void onStart() {
        cancelAll();
        currentPosition = fusedLocalizer.getPosition();

        schedule(
                new Sequential(
                        new Instant(this::requestOpModeStop)
                )
        );
    }

    @Override
    public void periodic() {
        CrossModeStorage.position = fusedLocalizer.getPosition();
        CrossModeStorage.covariance = fusedLocalizer.getCovariance();
    }

    private void update(Pair<Path, Path> path) {
        currentPosition = new Transform(path.second.position(0));
        paths.add(path.first);
        pathsHold.add(path.second);
    }

    private Path getPreviousPath(int i) {
        return paths.get(paths.size() - i);
    }

    private Pair<Path, Path> driveToCenter() {
        final double EASE_IN_FRACTION = 0.4;
        final double EASE_OUT_FRACTION = 0.4;

        return new PathBuilder(currentPosition.toList())
                .linearTo(new Transform(0, 0, 0).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.1, 50, true)
                .build();
    }

    private Command drive(Pair<Path, Path> path) {
        update(path);
        return followPath(path.first, 1);
    }

    private Command driveAndHold(Pair<Path, Path> path) {
        update(path);
        return new Sequential(
                followPath(path.first, 1),
                followPath(path.second, 1)
        );
    }

    private Command followPath(Path path, double multiplier) {
        return new Sequential(
                new HolonomicFollowPath(
                        new HolonomicPathTracker(
                                path,
                                new PIDF(List.of(
                                        new PIDFAxis(new PIDFAxis.K(Constants.pidPX * multiplier, Constants.pidIX * multiplier, Constants.pidDX * multiplier, new double[]{1, 0}, 6, 48, Constants.pidTau, Constants.pidGammaX)),
                                        new PIDFAxis(new PIDFAxis.K(Constants.pidPY * multiplier, Constants.pidIY * multiplier, Constants.pidDY * multiplier, new double[]{1, 0}, 6, 48, Constants.pidTau, Constants.pidGammaY)),
                                        new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta * multiplier, Constants.pidITheta * multiplier, Constants.pidDTheta * multiplier, new double[]{1, 0}, 6, 48, Constants.pidTau, Constants.pidGammaTheta))))),
                        fusedLocalizer, drivetrain)
                ,
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0)))
        );
    }
}