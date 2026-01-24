package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.First;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.RunUntil;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.Wait;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.pathbuilder.PathBuilder;
import org.beaverbots.beaver.pathing.pidf.PIDF;
import org.beaverbots.beaver.pathing.pidf.PIDFAxis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
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
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.function.ToDoubleFunction;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousNew extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer fusedLocalizer;
    private Shooter shooter;
    private VoltageSensor voltageSensor;
    private Intake intake;
    private Stopper stopper;

    private final Side side = Side.RED;
    private Motif motif;

    private List<DoubleUnaryOperator> mirror;

    private DrivetrainState currentPosition;

    private List<Path> paths = new ArrayList<>();
    private List<Path> pathsHold = new ArrayList<>();

    private ToDoubleFunction<Pair<List<Double>, List<Double>>> usageRatio;

    @Override
    public void onInit() {
        CrossModeStorage.side = side;
        mirror =
                side == Side.RED
                        ? List.of(x -> x, y -> y, theta -> theta)
                        : List.of(x -> x, y -> -y, theta -> -theta);

        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        limelight = new Limelight();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new DrivetrainState(0, 0, 0));
        voltageSensor = new VoltageSensor();
        shooter = new Shooter(voltageSensor);
        intake = new Intake(voltageSensor);
        stopper = new Stopper();

        usageRatio = PathBuilder.createHolonomicUsage(1 / Constants.drivetrainPowerConversionFactorX, 1 / Constants.drivetrainPowerConversionFactorY, 1 / Constants.drivetrainPowerConversionFactorTheta);

        register(gamepad, pinpoint, limelight, fusedLocalizer, voltageSensor, shooter, intake, stopper, drivetrain);
        limelight.localizationPipeline();

        schedule(
                new Repeat(() -> {
                    telemetry.addData("Position:", fusedLocalizer.getPosition().toString());
                    telemetry.addData("Variance X:", fusedLocalizer.getCovariance().getEntry(0, 0));
                    telemetry.addData("Variance Y:", fusedLocalizer.getCovariance().getEntry(1, 1));
                    telemetry.addData("Variance Theta:", fusedLocalizer.getCovariance().getEntry(2, 2));
                })
        );
    }

    @Override
    public void onStart() {
        cancelAll();
        currentPosition = fusedLocalizer.getPosition().transform(mirror);

        schedule(
                new Sequential(
                        shoot(driveToShootFar()),
                        intakeFrom(driveThroughSpike3()),
                        shoot(driveToShootFar()),
                        intakeFrom(driveThroughHumanPlayer()),
                        shoot(driveToShootFar()),
                        new Instant(() -> {
                            shooter.spin(0);
                        })
                )
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", fusedLocalizer.getPosition().toString());
        CrossModeStorage.position = fusedLocalizer.getPosition();
    }

    private PathBuilder newPathBuilder() {
        return new PathBuilder(currentPosition.toList(), mirror, false);
    }

    private PathBuilder newPathBuilderFromPath(Path path) {
        return new PathBuilder(path, mirror, false);
    }

    private void update(Pair<Path, Path> path) {
        currentPosition = new DrivetrainState(path.second.position(0)).transform(mirror);
        paths.add(path.first.transform(mirror));
        pathsHold.add(path.second.transform(mirror));
    }

    private Path getPreviousPath(int i) {
        return paths.get(paths.size() - i);
    }

    private Pair<Path, Path> driveToShootFar() {
        final double X = 54;
        final double Y = 18;

        final double EASING_FRACTION = 0.4;

        final DrivetrainState position = new DrivetrainState(
                X,
                Y,
                Localizer.wind(
                        Math.atan2(
                                Constants.GOAL_Y - Y,
                                Constants.GOAL_X - X
                        ) - Constants.shooterBias, currentPosition.getTheta()
                )
        );

        return newPathBuilder().linearTo(position.toList(), EASING_FRACTION, 1).stop(0.2, 0.2).retime(usageRatio, 1, 50).build();
    }

    private Pair<Path, Path> driveSplineToShootFar() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = 54;

        final double BEZIER_1_Y = 40;
        final double BEZIER_2_Y = 24;
        final double BEZIER_3_Y = 18;

        final double EASING_FRACTION = 0.4;

        DrivetrainState position1 = new DrivetrainState(currentPosition.getX(), BEZIER_1_Y, currentPosition.getTheta());
        DrivetrainState position2 = new DrivetrainState(currentPosition.getX(), BEZIER_2_Y, currentPosition.getTheta());
        DrivetrainState position3 = new DrivetrainState(X, BEZIER_3_Y, Localizer.wind(
                Math.atan2(
                        Constants.GOAL_Y - BEZIER_3_Y,
                        Constants.GOAL_X - X
                ) - Constants.shooterBias, currentPosition.getTheta()
        ));
        DrivetrainState position0 = new DrivetrainState(position1.toVector().mapMultiply(2).subtract(position2.toVector()));


        return newPathBuilder()
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), EASING_FRACTION, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), EASING_FRACTION, 1)
                .stop(0.2, 0.2)
                .retime(usageRatio, 1, 50)
                .build();
    }

    private Pair<Path, Path> driveThroughSpike1() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = 11.78125;

        final double BEZIER_1_Y = 28;
        final double BEZIER_2_Y = 40;
        final double BEZIER_3_Y = 56;

        final double EASING_FRACTION = 0.3;

        DrivetrainState position1 = new DrivetrainState(X, BEZIER_1_Y, Math.PI / 2);
        DrivetrainState position2 = new DrivetrainState(X, BEZIER_2_Y, Math.PI / 2);
        DrivetrainState position3 = new DrivetrainState(X, BEZIER_3_Y, Math.PI / 2);
        DrivetrainState position0 = new DrivetrainState(position1.toVector().mapMultiply(2).subtract(position2.toVector()));


        return newPathBuilder()
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), EASING_FRACTION, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), EASING_FRACTION, 1)
                .retime(usageRatio, 0.7, 50)
                .build();
    }

    private Pair<Path, Path> driveThroughSpike2() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = 11.78125;
        //final double X = 7;

        final double BEZIER_1_Y = 28;
        final double BEZIER_2_Y = 40;
        final double BEZIER_3_Y = 58;

        final double EASING_FRACTION = 0.3;


        DrivetrainState position1 = new DrivetrainState(X, BEZIER_1_Y, Math.PI / 2);
        DrivetrainState position2 = new DrivetrainState(X, BEZIER_2_Y, Math.PI / 2);
        DrivetrainState position3 = new DrivetrainState(X, BEZIER_3_Y, Math.PI / 2);
        DrivetrainState position0 = new DrivetrainState(position1.toVector().mapMultiply(2).subtract(position2.toVector()));


        return newPathBuilder()
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), EASING_FRACTION, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), EASING_FRACTION, 1)
                .retime(usageRatio, 0.7, 50)
                .build();
    }

    private Pair<Path, Path> driveThroughSpike3() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = 35.34375;

        final double BEZIER_1_Y = 18;
        final double BEZIER_2_Y = 30;
        final double BEZIER_3_Y = 56;

        final double EASING_FRACTION = 0.2; // It's longer time, so easing fraction will be larger proportional to the fraction

        DrivetrainState position1 = new DrivetrainState(X, BEZIER_1_Y, Math.PI / 2);
        DrivetrainState position2 = new DrivetrainState(X, BEZIER_2_Y, Math.PI / 2);
        DrivetrainState position3 = new DrivetrainState(X, BEZIER_3_Y, Math.PI / 2);
        DrivetrainState position0 = new DrivetrainState(position1.toVector().mapMultiply(2).subtract(position2.toVector()));


        return newPathBuilder()
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), EASING_FRACTION, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), EASING_FRACTION, 1)
                .retime(usageRatio, 0.7, 50)
                .build();
    }

    private Pair<Path, Path> driveThroughHumanPlayer() {
        final double X = 60 /* idk */;

        final double Y1 = 60;
        final double Y2 = 40;

        final double EASING_FRACTION = 1;

        DrivetrainState position1 = new DrivetrainState(X, Y1, Math.PI / 2);
        DrivetrainState position2 = new DrivetrainState(X, Y2, Math.PI / 2);
        DrivetrainState position3 = new DrivetrainState(X, Y1, Math.PI / 2);


        return newPathBuilder()
                .linearTo(position1.toList(), 0.5, 1, PathBuilder.EaseMode.PREEMPTIVE)
                .linearTo(position2.toList(), 0.5, 1)
                .linearTo(position3.toList(), 0.5, 1, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.5, 50)
                .build();
    }

    private Command shoot(Pair<Path, Path> path) {
        final Pair<Double, Double> shooterSettings = Shooter.getSettingsAtDistance(currentPosition.lateralDistance(new DrivetrainState(Constants.GOAL_X, Constants.GOAL_Y, 0)));
        final double MAX_RPM_ERROR = 50;
        final double MAX_ANGLE_ERROR = 0.02;

        update(path);

        return new Sequential(
                new Instant(() -> {
                    shooter.spin(shooterSettings.first);
                    shooter.setHood(shooterSettings.second);
                }),
                followPathTemplate(path.first),
                new RunUntil(
                        new Sequential(
                                new Wait(0.5),
                                new First(
                                        new WaitUntil(() ->
                                                shooter.getError() < MAX_RPM_ERROR
                                                && Math.abs(fusedLocalizer.getPosition().getTheta() - path.second.position(0).get(2)) < MAX_ANGLE_ERROR
                                        ),
                                        new Wait(10)
                                ),
                                new Wait(1),
                                new Instant(() -> {
                                    intake.spin(0.8);
                                    stopper.spin(1);
                                }),
                                new Wait(1)
                        ),
                        followPathTemplate(path.second)
                ),
                new Instant(() -> {
                    intake.spin(0);
                    stopper.spin(0);
                })
        );
    }

    private Command intakeFrom(Pair<Path, Path> path) {
        update(path);

        return new Sequential(
                new Instant(() -> {
                    intake.spin(1);
                    stopper.spin(-1);
                }),
                followPathTemplate(path.first),
                new Wait(0.3),
                new Instant(() -> {
                    intake.spin(0);
                    stopper.spin(0);
                })
        );
    }

    private Command leaveFar() {
        Pair<Path, Path> path = newPathBuilder().linearTo(new DrivetrainState(50, 50, currentPosition.getTheta()).toList(), 0.3, 5).build();
        update(path);

        return followPathTemplate(path.first);
    }

    private Command followPathTemplate(Path path) {
        return new Sequential(
                new HolonomicFollowPath(
                        path,
                        new PIDF(List.of(
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, Constants.pidTauX, Constants.pidGammaX)),
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, Constants.pidTauY, Constants.pidGammaY)),
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, Constants.pidTauTheta, Constants.pidGammaTheta)))),
                        fusedLocalizer, drivetrain),
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0))));
    }
}
