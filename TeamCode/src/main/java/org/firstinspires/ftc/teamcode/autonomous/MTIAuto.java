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
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.ArrayList;
import java.util.List;
import java.util.function.ToDoubleFunction;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MTIAuto extends CommandRuntimeOpMode {
    private GamepadEx gamepad;
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer fusedLocalizer;
    private VoltageSensor voltageSensor;
    private Intake intake;
    private Turret turret;

    private Transform currentPosition;

    private List<Path> paths = new ArrayList<>();
    private List<Path> pathsHold = new ArrayList<>();

    private ToDoubleFunction<Pair<List<Double>, List<Double>>> usageRatio;

    @Override
    public void onInit() {
        gamepad = new GamepadEx(gamepad1);
        drivetrain = new MecanumDrivetrain();
        pinpoint = new Pinpoint(new Transform(0, 0, 0));
        limelight = new Limelight();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new Transform(0, 0, 0));
        voltageSensor = new VoltageSensor();
        intake = new Intake();
        turret = new Turret(voltageSensor);

        usageRatio = PathBuilder.createHolonomicUsage(1 / Constants.drivetrainPowerConversionFactorX, 1 / Constants.drivetrainPowerConversionFactorY, 1 / Constants.drivetrainPowerConversionFactorTheta);

        register(gamepad, pinpoint, limelight, fusedLocalizer, voltageSensor, turret, intake, drivetrain);
        limelight.localizationPipeline();
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
                        shootNear(driveToShootNear()),
                        intakeFrom(driveThroughSpike2()),
                        shootNear(driveSplineToShootNear()),
                        intakeFrom(driveThroughSpike3()),
                        shootNear(driveSplineToShootNear()),
                        intakeFrom(driveThroughSpike1()),
                        shootNear(driveToShootNear()),
                        new Instant(() -> turret.shoot(0))
                )
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", fusedLocalizer.getPosition().toString());
        CrossModeStorage.position = fusedLocalizer.getPosition();
    }

    private void update(Pair<Path, Path> path) {
        currentPosition = new Transform(path.second.position(0));
        paths.add(path.first);
        pathsHold.add(path.second);
    }

    private Path getPreviousPath(int i) {
        return paths.get(paths.size() - i);
    }

    private Pair<Path, Path> driveToShootNear() {
        final Transform SHOOT_LOCATION = new Transform(-24, 26, 3 * Math.PI / 4);

        final double EASING_FRACTION = 0.4;

        return new PathBuilder(currentPosition.toList())
                .linearTo(SHOOT_LOCATION.toList(), EASING_FRACTION, 1)
                .stop(0.2, 0.2)
                .retime(usageRatio, 1, 50)
                .build();
    }

    private Pair<Path, Path> driveSplineToShootNear() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = -24;

        final double BEZIER_1_Y = 40;
        final double BEZIER_2_Y = 28;
        final double BEZIER_3_Y = 24;

        final double EASING_FRACTION = 0.4;

        Transform position1 = new Transform(currentPosition.getX(), BEZIER_1_Y, currentPosition.getTheta());
        Transform position2 = new Transform(currentPosition.getX(), BEZIER_2_Y, currentPosition.getTheta());
        Transform position3 = new Transform(X, BEZIER_3_Y, currentPosition.getTheta());
        Transform position0 = new Transform(position1.toVector().mapMultiply(2).subtract(position2.toVector()));


        return new PathBuilder(currentPosition.toList())
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), EASING_FRACTION, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), EASING_FRACTION, 1)
                .stop(0.2, 0.2)
                .retime(usageRatio, 1, 50)
                .build();
    }

    private Pair<Path, Path> driveThroughSpike1() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = -11.78125;

        final double BEZIER_1_Y = 28;
        final double BEZIER_2_Y = 40;
        final double BEZIER_3_Y = 56;

        final double EASING_FRACTION = 0.3;

        Transform position1 = new Transform(X, BEZIER_1_Y, Math.PI / 2);
        Transform position2 = new Transform(X, BEZIER_2_Y, Math.PI / 2);
        Transform position3 = new Transform(X, BEZIER_3_Y, Math.PI / 2);
        Transform position0 = new Transform(position1.toVector().mapMultiply(2).subtract(position2.toVector()));


        return new PathBuilder(currentPosition.toList())
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), EASING_FRACTION, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), EASING_FRACTION, 1)
                .retime(usageRatio, 0.7, 50)
                .build();
    }

    private Pair<Path, Path> driveThroughSpike2() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = 11.78125 + 3;
        //final double X = 7;

        final double BEZIER_1_Y = 28;
        final double BEZIER_2_Y = 40;
        final double BEZIER_3_Y = 58;

        final double EASING_FRACTION = 0.3;


        Transform position1 = new Transform(X, BEZIER_1_Y, Math.PI / 2);
        Transform position2 = new Transform(X, BEZIER_2_Y, Math.PI / 2);
        Transform position3 = new Transform(X, BEZIER_3_Y, Math.PI / 2);
        Transform position0 = new Transform(position1.toVector().mapMultiply(2).subtract(position2.toVector()));


        return new PathBuilder(currentPosition.toList())
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), EASING_FRACTION, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), EASING_FRACTION, 1)
                .retime(usageRatio, 0.7, 50)
                .build();
    }

    private Pair<Path, Path> driveThroughSpike3() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = 35.34375;

        final double BEZIER_1_Y = 28;
        final double BEZIER_2_Y = 40;
        final double BEZIER_3_Y = 56;

        final double EASING_FRACTION = 0.2; // It's longer time, so easing fraction will be larger proportional to the fraction

        Transform position1 = new Transform(X, BEZIER_1_Y, Math.PI / 2);
        Transform position2 = new Transform(X, BEZIER_2_Y, Math.PI / 2);
        Transform position3 = new Transform(X, BEZIER_3_Y, Math.PI / 2);
        Transform position0 = new Transform(position1.toVector().mapMultiply(2).subtract(position2.toVector()));


        return new PathBuilder(currentPosition.toList())
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), EASING_FRACTION, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), EASING_FRACTION, 1)
                .retime(usageRatio, 0.7, 50)
                .build();
    }

    private Command shootNear(Pair<Path, Path> path) {
        Transform goalLocation = new Transform(-72, 72);

        final double SHOOTER_RPM = 2800;
        final double MAX_ERROR = 100;

        update(path);

        return new RunUntil(
                        new Sequential(
                                new Instant(() -> turret.shoot(SHOOTER_RPM)),
                                followPathTemplate(path.first, 1),
                                new RunUntil(
                                        new Sequential(
                                                new Wait(1),
                                                new WaitUntil(() -> Math.abs(turret.getVelocity() - SHOOTER_RPM) < MAX_ERROR),
                                                new Instant(() -> {
                                                    intake.intake(true);
                                                    intake.transfer(true);
                                                }),
                                                new Wait(1)
                                        ),
                                        followPathTemplate(path.second, 1)
                                ),
                                new Instant(() -> intake.stop())
                        ),
                        new Repeat(() -> turret.turn(fusedLocalizer.getPosition().angleTo(goalLocation.subtract(fusedLocalizer.getVelocity().scale(0.7))) - fusedLocalizer.getPosition().getTheta()))
                );
    }

    private Command intakeFrom(Pair<Path, Path> path) {
        update(path);

        return new Sequential(
                new Instant(() -> {
                    intake.intake(true);
                    intake.transfer(false);
                }),
                followPathTemplate(path.first, 1),
                new Wait(0.3),
                new Instant(() -> intake.stop())
        );
    }


    private Command followPathTemplate(Path path, double multiplier) {
        return new Sequential(
                new HolonomicFollowPath(
                        path,
                        new PIDF(List.of(
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPX * multiplier, Constants.pidIX * multiplier, Constants.pidDX * multiplier, 1, 6, 48, Constants.pidTauX, Constants.pidGammaX)),
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPY * multiplier, Constants.pidIY * multiplier, Constants.pidDY * multiplier, 1, 6, 48, Constants.pidTauY, Constants.pidGammaY)),
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta * multiplier, Constants.pidITheta * multiplier, Constants.pidDTheta * multiplier, 1, 6, 48, Constants.pidTauTheta, Constants.pidGammaTheta)))),
                        fusedLocalizer, drivetrain),
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0)))
        );
    }
}