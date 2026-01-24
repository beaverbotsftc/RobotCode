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
public class AutonomousRed extends CommandRuntimeOpMode {
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
                new Sequential(
                        new RunUntil(
                                new WaitUntil(() -> gamepad.getDpadUpJustPressed()),
                                new Repeat(() -> {
                                    telemetry.addData("Position:", fusedLocalizer.getPosition().toString());
                                    telemetry.addData("Variance X:", fusedLocalizer.getCovariance().getEntry(0, 0));
                                    telemetry.addData("Variance Y:", fusedLocalizer.getCovariance().getEntry(1, 1));
                                    telemetry.addData("Variance Theta:", fusedLocalizer.getCovariance().getEntry(2, 2));
                                    telemetry.addLine("NOT READY TO START!!!");
                                })
                        ),
                        new Instant(() -> {
                            pinpoint.setPosition(fusedLocalizer.getPosition());
                            unregister(fusedLocalizer);
                            limelight.obeliskPipeline();
                        }),
                        new Repeat(() -> {
                            Motif result = limelight.getMotif(side);
                            if (result != null) motif = result;
                            telemetry.addData("Limelight now:", result);
                            telemetry.addData("Motif:", motif);
                            telemetry.addLine(limelight.getStatus().toString());
                            telemetry.addLine("Good to start");
                        })
                )
        );
    }

    @Override
    public void onStart() {
        cancelAll();
        currentPosition = pinpoint.getPosition().transform(mirror);

        schedule(
                new Sequential(
                        shootNear(driveToShootNear()),
                        intakeFrom(driveThroughSpike2()),
                        shootNear(driveSplineToShootNear()),
                        openGateNoPickup(driveToGateFront()),
                        intakeFrom(driveToIntakeGate()),
                        shootNear(driveSplineToShootNear()),
                        intakeFrom(driveThroughSpike1()),
                        shootNear(driveToShootNear()),
                        intakeFrom(driveThroughSpike3()),
                        shootNear(driveToShootNear()),
                        new Instant(() -> {
                            shooter.spin(0);
                        }),
                        leaveNear()
                        /*
                        shootNear(driveToShootNear()),
                        intakeSpike(driveThroughSpike1()),
                        openGateNoPickup(driveToGateSide()),
                        shootNear(driveToShootNear()),
                        intakeSpike(driveThroughSpike2()),
                        shootNear(newPathBuilderFromPath(getPreviousPath(1)).reverse().retime(usageRatio, 1, 50).build()),
                        intakeSpike(driveThroughSpike3()),
                        shootNear(driveToShootNear()),
                        new Instant(() -> {
                            shooter.spin(0);
                        }),
                        leaveNear()
                         */
                )
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", pinpoint.getPosition().toString());
        CrossModeStorage.position = pinpoint.getPosition();
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

    private Pair<Path, Path> driveToShootNear() {
        final double X = -24;//-16;
        final double Y = 24;

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

    private Pair<Path, Path> driveSplineToShootNear() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = -24;

        final double BEZIER_1_Y = 40;
        final double BEZIER_2_Y = 28;
        final double BEZIER_3_Y = 24;

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

    private Pair<Path, Path> driveToShootFar() {
        final double X = 60;
        final double Y = 18;

        final double EASING_FRACTION = 1;

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

        return newPathBuilder().linearTo(position.toList(), EASING_FRACTION, 1).retime(usageRatio, 0.7, 50).build();
    }

    private Pair<Path, Path> driveToGateSide() {
        final double X = 0;
        final double Y = 54;

        final double EASING_FRACTION = 1;

        final DrivetrainState position = new DrivetrainState(
                X,
                Y,
                Math.PI
        );

        return newPathBuilder()
                .linearTo(position.toList(), EASING_FRACTION, 1)
                .retime(usageRatio, 0.7, 50)
                .stop(1, 2)
                .build();
    }

    private Pair<Path, Path> driveToGateFront() {
        final double X = 6;

        final double BEZIER_1_Y = 28;
        final double BEZIER_2_Y = 40;
        final double BEZIER_3_Y = 52;

        final double EASING_FRACTION = 0.2;


        DrivetrainState position1 = new DrivetrainState(X, BEZIER_1_Y, Math.PI / 2);
        DrivetrainState position2 = new DrivetrainState(X, BEZIER_2_Y, Math.PI / 2);
        DrivetrainState position3 = new DrivetrainState(X, BEZIER_3_Y, Math.PI / 2);
        DrivetrainState position0 = new DrivetrainState(position1.toVector().mapMultiply(2).subtract(position2.toVector()));


        return newPathBuilder()
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), EASING_FRACTION, 1)
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), EASING_FRACTION, 1)
                .retime(usageRatio, 1, 50)
                .build();
    }

    private Pair<Path, Path> driveToIntakeGate() {
        final double X = 18;
        final double Y = 58;
        final double THETA = 2.3;

        final double EASING_FRACTION = 1;

        final DrivetrainState position = new DrivetrainState(
                X,
                Y,
                THETA
        );

        return newPathBuilder()
                .linearTo(position.toList(), EASING_FRACTION, 1)
                .retime(usageRatio, 0.3, 50)
                .stop(0.6, 1)
                .build();
    }

    private Pair<Path, Path> driveThroughSpike1() {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double X = -11.78125;

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
        final double X = 11.78125 + 1;
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

        final double BEZIER_1_Y = 28;
        final double BEZIER_2_Y = 40;
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
        final double X = 50 /* idk */;

        final double BEZIER_1_Y = 28;
        final double BEZIER_2_Y = 40;
        final double BEZIER_3_Y = 50 /* idk */;

        final double EASING_FRACTION = 1;

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

    private Command shootNear(Pair<Path, Path> path) {
        final double SHOOTER_RPM = 2200;
        final double MAX_ERROR = 50;
        final double HOOD_ANGLE = 0.3;

        update(path);

        return new Sequential(
                new Instant(() -> {
                    shooter.spin(SHOOTER_RPM);
                    shooter.setHood(HOOD_ANGLE);
                }),
                followPathTemplate(path.first),
                new RunUntil(
                        new Sequential(
                                new Wait(0.5),
                                new First(
                                        new WaitUntil(() -> shooter.getError() < MAX_ERROR),
                                        new Wait(10)
                                ),
                                new Instant(() -> {
                                    intake.spin(1);
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

    private Command shootFar(Pair<Path, Path> path) {
        final double SHOOTER_RPM = 3000;
        final double MAX_ERROR = 50;
        final double HOOD_ANGLE = 0.72;

        update(path);

        return new Sequential(
                new Instant(() -> {
                    shooter.spin(SHOOTER_RPM);
                    shooter.setHood(HOOD_ANGLE);
                }),
                followPathTemplate(path.first),
                new RunUntil(
                        new Sequential(
                                new Wait(0.5),
                                new First(
                                        new WaitUntil(() -> shooter.getError() < MAX_ERROR),
                                        new Wait(10)
                                ),
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

    private Command openGateNoPickup(Pair<Path, Path> path) {
        update(path);

        return followPathTemplate(path.first);
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

    private Command leaveNear() {
        final double X = 0;
        final double Y = 40;
        final double EASING_FRACTION = 0.2;

        DrivetrainState position = new DrivetrainState(X, Y, Math.PI / 2);

        Pair<Path, Path> path = newPathBuilder()
                .linearTo(position.toList(), EASING_FRACTION, 1)
                .stop(1, 1)
                .build();

        update(path);

        return new Sequential(
                followPathTemplate(path.first),
                followPathTemplate(path.second)
        );
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
