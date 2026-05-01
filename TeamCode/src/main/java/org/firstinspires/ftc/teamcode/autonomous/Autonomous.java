package org.firstinspires.ftc.teamcode.autonomous;

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
import org.beaverbots.beaver.util.Transform;
import org.beaverbots.beaver.util.Triple;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretControl;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.function.ToDoubleFunction;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AAAutonomous")
public class Autonomous extends CommandOpMode {
    private GamepadEx gamepad;
    private SwerveDrivetrain drivetrain;
    private Turret turret;
    private Intake intake;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer fusedLocalizer;
    private VoltageSensor voltageSensor;

    private Transform currentPosition;
    private Transform redCurrentPosition;

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
        intake = new Intake();

        usageRatio = PathBuilder.createHolonomicUsage(Constants.maxSpeedX, Constants.maxSpeedY, Constants.maxSpeedTheta, Constants.maxAccelerationX, Constants.maxAccelerationY, Constants.maxAccelerationTheta);

        register(voltageSensor, gamepad, pinpoint, limelight, fusedLocalizer);

        schedule(
                new Sequential(
                        new Wait(5),
                        new Instant(() -> register(drivetrain, turret, intake))
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

        mirror = CrossModeStorage.side == Side.RED
                ? new DoubleUnaryOperator[]{x -> x, y -> y, theta -> theta}
                : new DoubleUnaryOperator[]{x -> x, y -> -y, theta -> -theta};

        redCurrentPosition = fusedLocalizer.getPosition().transform(mirror);

        if (gamepad.getY())
            drivetrain.preempt(new Transform(driveFarLaunchLineInitial().first.velocity(0.1)).toLocalVelocity(currentPosition));
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
                        /*
                        shoot(driveLaunchLineInitial(), 2, 0.5, 0.5),
                        intake(driveSpike1(), 0.5),
                        driveAndPreepmt(driveGate(), 1),
                        shoot(driveLaunchLineStraight(), 0.5, 0.5, 0.5),
                        intake(driveSpike2(), 0.5),
                        shoot(driveLaunchLineSpline(), 0.5, 0.5, 0.5),
                        drive(driveLeave()),
                         */
                        shoot(driveFarLaunchLineInitial(), 2.0, 0.6, 0.3),
                        intake(driveHumanPlayer(0), 0.3),
                        shoot(driveFarLaunchLineInitial(), 0.2, 0.6, 0.3),
                        intake(driveSpike3(), 0.3),
                        shoot(driveFarLaunchLineInitial(), 0.2, 0.6, 0.3),
                        intake(driveHumanPlayer(0), 0.3),
                        shoot(driveFarLaunchLineInitial(), 0.2, 0.6, 0.3),
                        intake(driveHumanPlayer(24), 0.3),
                        shoot(driveFarLaunchLineInitial(), 0.2, 0.6, 0.3),
                        intake(driveHumanPlayer(0), 0.3),
                        shoot(driveFarLaunchLineInitial(), 0.2, 0.6, 0.3),
                        drive(driveLeaveFar()),
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
        redCurrentPosition = currentPosition.transform(mirror);
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

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .linearTo(new Transform(0, 0, 0).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.1, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveLeave() {
        final double EASE_IN_FRACTION = 0.0;
        final double EASE_OUT_FRACTION = 0.4;

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .linearTo(new Transform(0, 24, Math.PI / 2).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.5, 50, false)
                .build();
    }

    private Triple<Path, Path, Double> driveLeaveFar() {
        final double EASE_IN_FRACTION = 0.0;
        final double EASE_OUT_FRACTION = 0.4;

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .linearTo(new Transform(48, 24, Math.PI).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.5, 50, false)
                .build();
    }

    private Triple<Path, Path, Double> driveLaunchLineInitial() {
        final double EASE_IN_FRACTION = 0.5;
        final double EASE_OUT_FRACTION = 0.5;

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .linearTo(new Transform(-16, 20, Math.PI / 2).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.8, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveFarLaunchLineInitial() {
        final double EASE_IN_FRACTION = 0.0;
        final double EASE_OUT_FRACTION = 0.4;

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .linearTo(new Transform(62, 14, Math.PI / 2).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.8, 50, false)
                .build();
    }

    private Triple<Path, Path, Double> driveHumanPlayer(double offset) {
        final double EASE_IN_FRACTION = 0.0;
        final double EASE_OUT_FRACTION = 0.8;

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .linearTo(new Transform(64 - offset, 64, Math.PI / 2).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.8, 50, false)
                .build();
    }

    private Triple<Path, Path, Double> driveLaunchLineStraight() {
        final double EASE_IN_FRACTION = 0.5;
        final double EASE_OUT_FRACTION = 0.5;

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .linearTo(new Transform(-16, 20, Math.PI / 2).toList(), EASE_IN_FRACTION, 1)
                .stop(EASE_OUT_FRACTION, EASE_OUT_FRACTION, PathBuilder.EaseMode.PREEMPTIVE)
                .retime(usageRatio, 0.7, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveLaunchLineSpline() {
        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .c2BezierTo(List.of(
                        new Transform(redCurrentPosition.getX(), 20, currentPosition.getTheta()).toList(),
                        new Transform(-16, 20, Math.PI / 2).toList(),
                        new Transform(-16, 20, Math.PI / 2).toList(),
                        new Transform(-16, 20, Math.PI / 2).toList()
                ), 1)
                .retime(usageRatio, 0.7, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveGate() {
        final double Y1 = 35;
        final double X2 = -1;
        final double Y2 = 58; // Would be 54, but to ensure the open

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .c2BezierTo(List.of(
                        new Transform(redCurrentPosition.getX(), Y1, Math.PI / 2).toList(),
                        new Transform(redCurrentPosition.getX(), Y1, Math.PI / 2).toList(),
                        new Transform(X2, Y2, Math.PI / 2).toList(),
                        new Transform(X2, Y2, Math.PI / 2).toList(),
                        new Transform(X2, Y2, Math.PI / 2).toList()
                ), 1)
                .retime(usageRatio, 0.3, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveGateIntake() {
        final double X1 = 9;
        final double Y1 = 55; // Would be 53, but to ensure the open
        final double X2 = 14;
        final double Y2 = 60; // Would be 58, but to ensure the open
        final double THETA2 = 2.15;

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .c2BezierTo(List.of(
                        new Transform(X1, Y1, Math.PI / 2).toList(),
                        new Transform(X1, Y1, Math.PI / 2).toList(),
                        new Transform(X1, Y1, Math.PI / 2).toList(),
                        new Transform(X1, Y1, Math.PI / 2).toList(),
                        new Transform(X2, Y2, THETA2).toList(),
                        new Transform(X2, Y2, THETA2).toList(),
                        new Transform(X2, Y2, THETA2).toList()
                ), 1)
                .retime(usageRatio, 0.9, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveSpike1() {
        final double X = -11.78125;
        final double Y = 54;

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .c2BezierTo(List.of(
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList()
                ), 1)
                .retime(usageRatio, 0.4, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveSpike2() {
        final double X = 14;
        final double Y = 60;

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .c2BezierTo(List.of(
                        new Transform((2 * redCurrentPosition.getX() + X) / 3, redCurrentPosition.getY(), Math.PI / 2).toList(),
                        new Transform((redCurrentPosition.getX() + 2 * X) / 3, redCurrentPosition.getY(), Math.PI / 2).toList(),
                        new Transform(X, 20, Math.PI / 2).toList(),
                        new Transform(X, 20, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList()
                ), 1)
                .retime(usageRatio, 0.6, 50, true)
                .build();
    }

    private Triple<Path, Path, Double> driveSpike3() {
        final double X = 35.28125;
        final double Y = 60;

        return new PathBuilder(redCurrentPosition.toList(), mirror, false)
                .c2BezierTo(List.of(
                        new Transform(X, redCurrentPosition.getY(), Math.PI / 2).toList(),
                        new Transform(X, redCurrentPosition.getY(), Math.PI / 2).toList(),
                        new Transform(X, redCurrentPosition.getY(), Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList(),
                        new Transform(X, Y, Math.PI / 2).toList()
                ), 1)
                .retime(usageRatio, 0.6, 50, false)
                .build();
    }

    private Command preempt() {
        final int currentPathCaptured = paths.size() - 1;
        final Transform currentPositionCaptured = redCurrentPosition;
        return new Sequential(
                new Instant(() -> drivetrain.move(Transform.ZERO)),
                new Instant(() ->
                        drivetrain.preempt(
                                (new Transform(paths.get(currentPathCaptured + 1).velocity(0.01)).multiply(new Transform(Constants.pidFVelocityX, Constants.pidFVelocityY, Constants.pidFVelocityTheta))
                                        .add(new Transform(paths.get(currentPathCaptured + 1).acceleration(0.01))).multiply(new Transform(Constants.pidFAccelerationX, Constants.pidFAccelerationY, Constants.pidFAccelerationTheta))).toLocalVelocity(redCurrentPosition)
                        )
                )
        );
    }

    private Command drive(Triple<Path, Path, Double> path) {
        update(path);
        return followPath(path.first, 1);
    }

    private Command driveAndPreepmt(Triple<Path, Path, Double> path, double settlingTime) {
        update(path);
        return new Sequential(
                followPath(path.first, 1),
                preempt(),
                new Wait(settlingTime)
        );
    }

    private Command intake(Triple<Path, Path, Double> path, double settlingTime) {
        return new Sequential(
                new Instant(() -> intake.transfer(false)),
                new Instant(() -> intake.intake(1)),
                drive(path),
                preempt(),
                new Wait(settlingTime),
                new Instant(() -> intake.intake(0))
        );
    }

    private Command intakeFromGate(Triple<Path, Path, Double> path, double settlingTime) {
        update(path);
        return new Sequential(
                new Instant(() -> intake.transfer(false)),
                new Instant(() -> intake.intake(1)),
                drive(path),
                preempt(),
                new Wait(settlingTime),
                new Instant(() -> intake.intake(0))
        );
    }

    private Command shoot(Triple<Path, Path, Double> path, double shooterSettlingTime, double shootingTime, double preepmtionTime) {
        final List<Double> launchZoneX = List.of(0.0, -72.0, -72.0);
        final List<Double> launchZoneY = List.of(0.0, -72.0, 72.0);
        final List<Double> farLaunchZoneX = List.of(48.0, 72.0, 72.0);
        final List<Double> farLaunchZoneY = List.of(0.0, 24.0, -24.0);

        update(path);
        return new Sequential(
                new Parallel(
                        new Instant(() -> intake.intake(0.5)),
                        new Instant(() -> intake.transfer(false)),
                        new Sequential(
                                new WaitUntil(() -> {
                                    Pair<List<Double>, List<Double>> robot = Geometry.generateBox(fusedLocalizer.getPosition().getX(), fusedLocalizer.getPosition().getY(), 16, 18, fusedLocalizer.getPosition().getTheta());
                                    return Geometry.polygonPolygonIntersects(launchZoneX, launchZoneY, robot.first, robot.second) || Geometry.polygonPolygonIntersects(farLaunchZoneX, farLaunchZoneY, robot.first, robot.second);
                                }),
                                new RunUntil(
                                        new Sequential(
                                                new Wait(shooterSettlingTime),
                                                new Instant(() -> intake.transfer(true)),
                                                new Wait(shootingTime)
                                        ),
                                        new TurretControl(turret, fusedLocalizer, mirror)
                                )
                        ),
                        new Sequential(
                                drive(path),
                                preempt(),
                                new Wait(preepmtionTime)
                        )
                ),
                new Instant(() -> intake.transfer(false)),
                new Instant(() -> intake.intake(0))
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