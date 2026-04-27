package org.firstinspires.ftc.teamcode.autonomous;

import org.beaverbots.beaver.command.premade.Defer;
import org.beaverbots.beaver.command.premade.First;
import org.beaverbots.beaver.util.Pair;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Parallel;
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
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.util.Transform;
import org.beaverbots.beaver.util.Triple;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretControl;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleUnaryOperator;
import java.util.function.ToDoubleFunction;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AAAutonomous")
public class Autonomous extends CommandOpMode {
    private GamepadEx gamepad;
    private SwerveDrivetrain drivetrain;
    private Turret turret;
    private IntakeAndTransfer intakeAndTransfer;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer fusedLocalizer;
    private VoltageSensor voltageSensor;

    private Transform currentPosition;

    private List<Path> paths = new ArrayList<>();
    private List<Path> pathsHold = new ArrayList<>();
    private List<Double> times = new ArrayList<>();

    private DoubleUnaryOperator[] mirror;

    private ToDoubleFunction<Triple<List<Double>, List<Double>, List<Double>>> usageRatio;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        gamepad = new GamepadEx(gamepad1);
        pinpoint = new Pinpoint(new Transform(0, 0, 0));
        limelight = new Limelight(Limelight.Pipeline.LOCALIZATION_GOAL);
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new Transform(0, 0, 0));
        drivetrain = new SwerveDrivetrain(voltageSensor, false);
        turret = new Turret(voltageSensor);
        intakeAndTransfer = new IntakeAndTransfer();

        usageRatio = PathBuilder.createHolonomicUsage(Constants.maxSpeedX, Constants.maxSpeedY, Constants.maxSpeedTheta, Constants.maxAccelerationX, Constants.maxAccelerationY, Constants.maxAccelerationTheta);

        register(voltageSensor, gamepad, pinpoint, limelight, fusedLocalizer);

        schedule(
                new Sequential(
                        new Wait(5),
                        new Instant(() -> register(drivetrain, turret, intakeAndTransfer))
                )
        );
    }

    public void periodicInit() {
        telemetry.addData("Position:", fusedLocalizer.getPosition().toString());
        telemetry.addData("Variance X:", fusedLocalizer.getCovariance().getEntry(0, 0));
        telemetry.addData("Variance Y:", fusedLocalizer.getCovariance().getEntry(1, 1));
        telemetry.addData("Variance Theta:", fusedLocalizer.getCovariance().getEntry(2, 2));

        if (gamepad.getX()) CrossModeStorage.side = Side.BLUE;
        if (gamepad.getB()) CrossModeStorage.side = Side.RED;
        addData("Side", CrossModeStorage.side);
        currentPosition = fusedLocalizer.getPosition();

        if (gamepad.getY())
            drivetrain.preempt(new Transform(driveLaunchLineInitial().first.velocity(0.1)).toLocalVelocity(currentPosition));
    }

    @Override
    public void onStart() {
        currentPosition = fusedLocalizer.getPosition();
        mirror = CrossModeStorage.side == Side.RED
                ? new DoubleUnaryOperator[]{x -> x, y -> y, theta -> theta}
                : new DoubleUnaryOperator[]{x -> x, y -> -y, theta -> -theta};

        cancelAll();
        schedule(
                new Sequential(
                        shoot(driveLaunchLineInitial(), 2.5),
                        intake(driveSpike1()),
                        drive(driveGate()),
                        shoot(driveLaunchLine(), 0.5),
                        intake(driveSpike2()),
                        shoot(new PathBuilder(getPreviousPath(1)).reverse().build(), 0.5),
                        intakeFromGate(driveGateIntake()),
                        shoot(driveLaunchLineFromGate(), 0.5),
                        intakeFromGate(driveGateIntake()),
                        shoot(driveLaunchLineFromGate(), 0.5),
                        driveWithoutPreemption(driveLeave()),
                        new Instant(this::requestOpModeStop)
                )
        );
    }

    @Override
    public void periodic() {
        CrossModeStorage.position = fusedLocalizer.getPosition();
        CrossModeStorage.covariance = fusedLocalizer.getCovariance();
    }

    private void update(Triple<Path, Path, Double> path) {
        currentPosition = new Transform(path.second.position(0));
        paths.add(path.first);
        pathsHold.add(path.second);
        times.add(path.third);
    }

    private Path getPreviousPath(int i) {
        return paths.get(paths.size() - i);
    }

    private Triple<Path, Path, Double> driveToCenter() {
        final double EASE_IN_FRACTION = 0.4;
        final double EASE_OUT_FRACTION = 0.4;

        return new PathBuilder(currentPosition.toList())
                .linearTo(new Transform(0, 0, 0).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.1, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveLeave() {
        final double EASE_IN_FRACTION = 0.4;
        final double EASE_OUT_FRACTION = 0.4;

        return new PathBuilder(currentPosition.toList())
                .linearTo(new Transform(0, 24, Math.PI / 2).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.5, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveLaunchLineInitial() {
        final double EASE_IN_FRACTION = 0.3;
        final double EASE_OUT_FRACTION = 0.3;

        return new PathBuilder(currentPosition.toList())
                .linearTo(new Transform(-16, 20, Math.PI / 2).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.5, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveLaunchLine() {
        final double EASE_IN_FRACTION = 0.5;
        final double EASE_OUT_FRACTION = 0.5;

        return new PathBuilder(currentPosition.toList())
                .linearTo(new Transform(-16, 20, Math.PI / 2).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.5, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveLaunchLineFromGate() {
        return new PathBuilder(currentPosition.toList())
                .c2BezierTo(List.of(
                        new Transform(currentPosition.getX(), 20, currentPosition.getTheta()).toList(),
                        new Transform(-16, 20, Math.PI / 2).toList(),
                        new Transform(-16, 20, Math.PI / 2).toList(),
                        new Transform(-16, 20, Math.PI / 2).toList()
                ), 1)
                .retime(usageRatio, 0.5, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveGate() {
        return new PathBuilder(currentPosition.toList())
                .c2BezierTo(List.of(
                        new Transform(currentPosition.getX(), 25, Math.PI / 2).toList(),
                        new Transform(currentPosition.getX(), 25, Math.PI / 2).toList(),
                        new Transform(-1, 58, Math.PI / 2).toList(),
                        new Transform(-1, 58, Math.PI / 2).toList(),
                        new Transform(-1, 58, Math.PI / 2).toList()
                ), 1)
                .retime(usageRatio, 0.5, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveGateIntake() {
        /*
        final double X = 11;
        final double Y = 62;
        final double THETA = 2.1;

        return new PathBuilder(currentPosition.toList())
                .c2BezierTo(List.of(
                        new Transform((currentPosition.getX() + X) / 2, currentPosition.getY(), (currentPosition.getTheta() + THETA) / 2).toList(),
                        new Transform(X, currentPosition.getY(), THETA).toList(),
                        new Transform(X, Y, THETA).toList(),
                        new Transform(X, Y, THETA).toList(),
                        new Transform(X, Y, THETA).toList()
                ), 1)
                .retime(usageRatio, 0.4, 50, true)
                .build();
         */

        final double X1 = 8;
        final double Y1 = 56; // The physical max is 52, but bezier curves for you I guess.
        final double X2 = 20;
        final double Y2 = 56;
        final double THETA2 = 2.2;

        return new PathBuilder(currentPosition.toList())
                .c2BezierTo(List.of(
                        new Transform(X1, Y1, Math.PI / 2).toList(),
                        new Transform(X1, Y1, Math.PI / 2).toList(),
                        new Transform(X1, Y1, Math.PI / 2).toList(),
                        new Transform(X1, Y1, Math.PI / 2).toList(),
                        new Transform(X2, Y2, THETA2).toList(),
                        new Transform(X2, Y2, THETA2).toList(),
                        new Transform(X2, Y2, THETA2).toList()
                ), 1)
                .retime(usageRatio, 0.6, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveSpike1() {
        final double X = -11.78125;
        final double Y = 52.281250;

        return new PathBuilder(currentPosition.toList())
                .c2BezierTo(List.of(
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList()
                ), 1)
                .retime(usageRatio, 0.6, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveSpike2() {
        final double X = 11.78125;
        final double Y = 62;

        return new PathBuilder(currentPosition.toList())
                .c2BezierTo(List.of(
                        new Transform((2 * currentPosition.getX() + X) / 3, currentPosition.getY(), Math.PI / 2).toList(),
                        new Transform((currentPosition.getX() + 2 * X) / 3, currentPosition.getY(), Math.PI / 2).toList(),
                        new Transform(X, 20, Math.PI / 2).toList(),
                        new Transform(X, 20, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList()
                ), 1)
                .retime(usageRatio, 0.8, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveSpike3() {
        final double X = 35.34375;
        final double Y = 52.281250;

        return new PathBuilder(currentPosition.toList())
                .c2BezierTo(List.of(
                        new Transform((3 * currentPosition.getX() + X) / 4, currentPosition.getY(), Math.PI / 2).toList(),
                        new Transform((2 * currentPosition.getX() + 2 * X) / 4, currentPosition.getY(), Math.PI / 2).toList(),
                        new Transform((currentPosition.getX() + 3 * X) / 4, currentPosition.getY(), Math.PI / 2).toList(),
                        new Transform(X, 20, Math.PI / 2).toList(),
                        new Transform(X, 20, Math.PI / 2).toList(),
                        new Transform(X, 20, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList()
                ), 1)
                .retime(usageRatio, 0.5, 50, true)
                .build();
    }

    private Command drive(Triple<Path, Path, Double> path) {
        update(path);
        Stopwatch stopwatch = new Stopwatch();
        int currentPath = paths.size() - 1;
        return new Sequential(
                new Instant(stopwatch::reset),
                new First(
                        followPath(path.first, 1),
                        new Sequential(
                                // Last quarter or 1s, whichever is less, is fair game.
                                new WaitUntil(() -> path.third - stopwatch.getElapsed() < Math.min(1, path.third / 4) && fusedLocalizer.getVelocity().norm(Constants.pidFVelocityX / Constants.pidFVelocityTheta) < 4),
                                // Works because it runs top to bottom, First that is.
                                new Instant(() -> drivetrain.move(Transform.ZERO)),
                                new Instant(() -> drivetrain.preempt(new Transform(paths.get(currentPath + 1).velocity(0.001))))
                        )
                ),
                new Defer(() -> new Wait(path.third - stopwatch.getElapsed()))
        );
    }

    private Command driveWithoutPreemption(Triple<Path, Path, Double> path) {
        update(path);
        return followPath(path.first, 1);
    }

    private Command intake(Triple<Path, Path, Double> path) {
        return new Sequential(
                new Instant(() -> intakeAndTransfer.transfer(false)),
                new Instant(() -> intakeAndTransfer.intake(1)),
                drive(path),
                new Instant(() -> intakeAndTransfer.intake(0))
        );
    }

    private Command intakeFromGate(Triple<Path, Path, Double> path) {
        update(path);
        return new Sequential(
                new Instant(() -> intakeAndTransfer.transfer(false)),
                new Instant(() -> intakeAndTransfer.intake(1)),
                drive(path),
                new RunUntil(
                        new Wait(2),
                        hold(path)
                ),
                new Instant(() -> intakeAndTransfer.intake(0))
        );
    }

    private Command shoot(Triple<Path, Path, Double> path, double settlingTime) {
        final List<Double> launchZoneX = List.of(0.0, -72.0, -72.0);
        final List<Double> launchZoneY = List.of(0.0, -72.0, 72.0);

        update(path);
        return new Sequential(
                new Parallel(
                        drive(path),
                        new Instant(() -> intakeAndTransfer.intake(1)),
                        new Instant(() -> intakeAndTransfer.transfer(false)),
                        new Sequential(
                                new WaitUntil(() -> {
                                    Pair<List<Double>, List<Double>> robot = Geometry.generateBox(fusedLocalizer.getPosition().getX(), fusedLocalizer.getPosition().getY(), 16, 18, fusedLocalizer.getPosition().getTheta());
                                    return Geometry.polygonPolygonIntersects(launchZoneX, launchZoneY, robot.first, robot.second);
                                }),
                                new RunUntil(
                                        new Sequential(
                                                new Wait(settlingTime),
                                                new Instant(() -> intakeAndTransfer.transfer(true)),
                                                new Wait(0.5)
                                        ),
                                        new TurretControl(turret, fusedLocalizer, mirror)
                                )
                        )
                ),
                new Instant(() -> intakeAndTransfer.transfer(false)),
                new Instant(() -> intakeAndTransfer.intake(0))
        );
    }

    private Command hold(Triple<Path, Path, Double> path) {
        return followPath(path.second, 1);
    }

    private Command followPath(Path path, double multiplier) {
        HolonomicPathTracker tracker = new HolonomicPathTracker(
                path,
                new PIDF(List.of(
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPX * multiplier, Constants.pidIX * multiplier, Constants.pidDX * multiplier, new double[]{Constants.pidFVelocityX, Constants.pidFAccelerationX}, 6, 48, Constants.pidTau, Constants.pidGammaX, 0.1)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPY * multiplier, Constants.pidIY * multiplier, Constants.pidDY * multiplier, new double[]{Constants.pidFVelocityY, Constants.pidFAccelerationY}, 6, 48, Constants.pidTau, Constants.pidGammaY, 0.1)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta * multiplier, Constants.pidITheta * multiplier, Constants.pidDTheta * multiplier, new double[]{Constants.pidFVelocityTheta, Constants.pidFAccelerationTheta}, 6, 48, Constants.pidTau, Constants.pidGammaTheta, 0.1)))));
        return new RunUntil(new Sequential(
                new HolonomicFollowPath(
                        tracker,
                        fusedLocalizer, drivetrain)
                ,
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0)))
        ),
                new Repeat(() -> {
                    double x = tracker.getPosition().get(0);
                    double y = tracker.getPosition().get(1);
                    double theta = tracker.getPosition().get(2);

                    final double w = 16;
                    final double h = 18;

                    double halfW = w / 2.0;
                    double halfH = h / 2.0;

                    double cos = Math.cos(theta);
                    double sin = Math.sin(theta);

// local robot corners (forward = +x axis of robot)
                    double[] lx = {halfH, halfH, -halfH, -halfH};
                    double[] ly = {halfW, -halfW, -halfW, halfW};

                    double[] xPoints = new double[4];
                    double[] yPoints = new double[4];

                    for (int i = 0; i < 4; i++) {
                        xPoints[i] = x + (lx[i] * cos - ly[i] * sin);
                        yPoints[i] = y + (lx[i] * sin + ly[i] * cos);
                    }

                    packet.fieldOverlay()
                            .setFill("purple")
                            .fillPolygon(xPoints, yPoints);
                })
        );
    }
}