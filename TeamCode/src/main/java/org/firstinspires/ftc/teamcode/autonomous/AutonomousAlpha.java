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
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.GateOpener;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Transform;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.function.ToDoubleFunction;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousAlpha extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer fusedLocalizer;
    private Shooter shooter;
    private VoltageSensor voltageSensor;
    private Intake intake;
    private Stopper stopper;
    private GateOpener gateOpener;

    private final Side side = Side.RED;
    private Motif motif;

    private List<DoubleUnaryOperator> mirror;

    private Transform currentPosition;

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
        pinpoint = new Pinpoint(new Transform(0, 0, 0));
        limelight = new Limelight();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new Transform(0, 0, 0));
        voltageSensor = new VoltageSensor();
        shooter = new Shooter(voltageSensor);
        intake = new Intake(1);
        stopper = new Stopper();
        gateOpener = new GateOpener();

        usageRatio = PathBuilder.createHolonomicUsage(1 / Constants.drivetrainPowerConversionFactorX, 1 / Constants.drivetrainPowerConversionFactorY, 1 / Constants.drivetrainPowerConversionFactorTheta);

        register(gamepad, pinpoint, limelight, fusedLocalizer, voltageSensor, shooter, intake, stopper, drivetrain, gateOpener);
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
        Stopwatch s = new Stopwatch();

        schedule(
                new Repeat(() -> {
                    telemetry.addData("dt", s.getDt());
                }),
                new Sequential(
                        new Instant(() -> {
                            shooter.setFlywheelSpeed(2100);
                            shooter.setHoodAngle(0.1);
                        }),
                        new Instant(() -> gateOpener.close()),
                        shoot(lineToShootNear(), 0.75),
                        intake(splineThroughSpike2()),
                        shoot(splineToShootNear(), 0.75),
                        intakeFromGate(splineToIntakeGate(), 1.2),
                        shoot(splineToShootNear(), 0.75),
                        intakeFromGate(splineToIntakeGate(), 2),
                        shoot(splineToShootNear(), 0.75),
                        intake(splineThroughSpike1()),
                        shoot(splineToShootNear(), 0.75),
                        new Instant(this::requestOpModeStop)
                )
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", fusedLocalizer.getPosition().toString());
        telemetry.addData("Variance X:", fusedLocalizer.getCovariance().getEntry(0, 0));
        telemetry.addData("Variance Y:", fusedLocalizer.getCovariance().getEntry(1, 1));
        telemetry.addData("Variance Theta:", fusedLocalizer.getCovariance().getEntry(2, 2));
        CrossModeStorage.position = fusedLocalizer.getPosition();
    }

    private PathBuilder newPathBuilder() {
        return new PathBuilder(currentPosition.toList(), mirror, false);
    }

    private PathBuilder newPathBuilderFromPath(Path path) {
        return new PathBuilder(path, mirror, false);
    }

    private void update(Pair<Path, Path> path) {
        currentPosition = new Transform(path.second.position(0)).transform(mirror);
        paths.add(path.first.transform(mirror));
        pathsHold.add(path.second.transform(mirror));
    }

    private Path getPreviousPath(int i) {
        return paths.get(paths.size() - i);
    }

    private Command drive(Pair<Path, Path> path) {
        update(path);
        return followPathTemplate(path.first);
    }

    private Command driveAndStop(Pair<Path, Path> path) {
        update(path);
        return new Sequential(
                followPathTemplate(path.first),
                followPathTemplate(path.second)
        );
    }

    private Command shoot(Pair<Path, Path> path, double timeout) {
        update(path);
        return new Sequential(
                followPathTemplate(path.first),
                new RunUntil(
                        new Sequential(
                                new Wait(0.5),
                                new WaitUntil(() -> Math.abs(shooter.getError()) < 80),
                                new Instant(() -> intake.spin(1)),
                                new Instant(() -> stopper.spin(1)),
                                new First(
                                        new Wait(timeout)
                                        /*new Sequential(
                                                new WaitUntil(() -> intake.isEmpty()),
                                                new Wait(0.2)
                                        )*/
                                )
                        ),
                        followPathTemplate(path.second)
                ),
                new Instant(() -> intake.spin(0)),
                new Instant(() -> stopper.spin(0))
        );
    }

    private Command intake(Pair<Path, Path> path) {
        update(path);
        return new Sequential(
                new Instant(() -> intake.spin(1)),
                new Instant(() -> stopper.spin(-1)),
                followPathTemplate(path.first),
                new Wait(0.2),
                new Instant(() -> intake.spin(0)),
                new Instant(() -> stopper.spin(0))
        );
    }


    private Command intakeFromGate(Pair<Path, Path> path, double timeout) {
        update(path);
        return new Sequential(
                new Instant(() -> intake.spin(1)),
                new Instant(() -> stopper.spin(-1)),
                new Instant(() -> gateOpener.open()),
                followPathTemplate(path.first),
                new RunUntil(
                        new First(
                                //new WaitUntil(() -> intake.isFull()),
                                new Wait(timeout)
                        ),
                        followPathTemplate(path.second)
                ),
                new Instant(() -> intake.spin(0)),
                new Instant(() -> stopper.spin(0)),
                new Instant(() -> gateOpener.close())
        );
    }

    public Pair<Path, Path> lineToShootNear() {
        final double X = -24;
        final double Y = 24;

        final double EASING_FRACTION = 0.4;
        final double STOPPING_FRACTION = 0.2;

        final Transform position = new Transform(
                X,
                Y,
                Localizer.wind(
                        Math.atan2(
                                Constants.GOAL_Y - Y,
                                Constants.GOAL_X - X
                        ) - Constants.shooterBias, currentPosition.getTheta()
                )
        );

        return newPathBuilder()
                .linearTo(position.toList(), EASING_FRACTION, 1)
                .stop(STOPPING_FRACTION, STOPPING_FRACTION)
                .retime(usageRatio, 1, 50)
                .build();
    }


    private Pair<Path, Path> splineToShootNear() {
        final double X = -24;

        final double BEZIER_1_Y = 40;
        final double BEZIER_2_Y = 28;
        final double BEZIER_3_Y = 24;

        final double EASING_FRACTION = 0.4;
        final double STOPPING_FRACTION = 0.2;
        final double BACKUP_FRACTION = 1;

        Transform position1 = new Transform(currentPosition.getX(), BEZIER_1_Y, currentPosition.getTheta());
        Transform position2 = new Transform(currentPosition.getX(), BEZIER_2_Y, currentPosition.getTheta());
        Transform position3 = new Transform(X, BEZIER_3_Y, Localizer.wind(
                Math.atan2(
                        Constants.GOAL_Y - BEZIER_3_Y,
                        Constants.GOAL_X - X
                ) - Constants.shooterBias, currentPosition.getTheta()
        ));
        Transform position0 = new Transform(position1.toVector().mapMultiply(1 + BACKUP_FRACTION).subtract(position2.toVector().mapMultiply(BACKUP_FRACTION)));


        return newPathBuilder()
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), 0, BACKUP_FRACTION)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), 0, 1)
                .stop(STOPPING_FRACTION, STOPPING_FRACTION)
                .retime(usageRatio, 1, 50)
                .build();
    }

    private Pair<Path, Path> splineThroughSpike1() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = -11.78125;

        final double BEZIER_1_Y = 28;
        final double BEZIER_2_Y = 35;
        final double BEZIER_3_Y = 60;

        final double EASING_FRACTION = 0.3;
        final double STOPPING_FRACTION = 0;
        final double INTAKE_FRACTION = 2.6;

        Transform position1 = new Transform(X, BEZIER_1_Y,
                Localizer.wind(
                        Math.PI / 2,
                        currentPosition.getTheta()
                )
        );
        Transform position2 = new Transform(X, BEZIER_2_Y,
                Localizer.wind(
                        Math.PI / 2,
                        currentPosition.getTheta()
                )
        );
        Transform position3 = new Transform(X, BEZIER_3_Y,
                Localizer.wind(
                        Math.PI / 2,
                        currentPosition.getTheta()
                )
        );
        Transform position0 = new Transform(position1.toVector().mapMultiply(1 + 1 / INTAKE_FRACTION).subtract(position2.toVector().mapMultiply(1 / INTAKE_FRACTION)));


        return newPathBuilder()
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), 0, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), 0, INTAKE_FRACTION)
                .stop(STOPPING_FRACTION, STOPPING_FRACTION)
                .retime(usageRatio, 0.8, 50)
                .build();
    }

    private Pair<Path, Path> splineThroughSpike2() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = 11.78125 + 2;

        final double BEZIER_1_Y = 28;
        final double BEZIER_2_Y = 45;
        final double BEZIER_3_Y = 64;

        final double EASING_FRACTION = 0.3;
        final double STOPPING_FRACTION = 0;
        final double INTAKE_FRACTION = 1.2;

        Transform position1 = new Transform(X, BEZIER_1_Y,
                Localizer.wind(
                        Math.PI / 2,
                        currentPosition.getTheta()
                )
        );
        Transform position2 = new Transform(X, BEZIER_2_Y,
                Localizer.wind(
                        Math.PI / 2,
                        currentPosition.getTheta()
                )
        );
        Transform position3 = new Transform(X, BEZIER_3_Y,
                Localizer.wind(
                        Math.PI / 2,
                        currentPosition.getTheta()
                )
        );
        Transform position0 = new Transform(position1.toVector().mapMultiply(1 + 1 / INTAKE_FRACTION).subtract(position2.toVector().mapMultiply(1 / INTAKE_FRACTION)));


        return newPathBuilder()
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), 0, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), 0, INTAKE_FRACTION)
                .stop(STOPPING_FRACTION, STOPPING_FRACTION)
                .retime(usageRatio, 0.8, 50)
                .build();
    }

    private Pair<Path, Path> splineToIntakeGate() {
        final double X = 12.2328 + 1;

        final double BEZIER_1_Y = 28;
        final double BEZIER_2_Y = 40;
        final double BEZIER_3_Y = 55.9232 + 3.5;

        final double THETA = 1.9177;

        final double EASING_FRACTION = 0.3;
        final double STOPPING_FRACTION = 0;
        final double INTAKE_FRACTION = 2;

        Transform position1 = new Transform(X, BEZIER_1_Y,
                Localizer.wind(
                        THETA,
                        currentPosition.getTheta()
                )
        );
        Transform position2 = new Transform(X, BEZIER_2_Y,
                Localizer.wind(
                        THETA,
                        currentPosition.getTheta()
                )
        );
        Transform position3 = new Transform(X, BEZIER_3_Y,
                Localizer.wind(
                        THETA,
                        currentPosition.getTheta()
                )
        );
        Transform position0 = new Transform(position1.toVector().mapMultiply(1 + 1 / INTAKE_FRACTION).subtract(position2.toVector().mapMultiply(1 / INTAKE_FRACTION)));


        return newPathBuilder()
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), 0, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), 0, INTAKE_FRACTION)
                .stop(STOPPING_FRACTION, STOPPING_FRACTION)
                .retime(usageRatio, 0.8, 50)
                .build();
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
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0))));
    }
}
