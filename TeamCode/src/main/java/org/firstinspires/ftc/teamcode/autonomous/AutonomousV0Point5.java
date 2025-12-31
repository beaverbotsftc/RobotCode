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
import org.beaverbots.beaver.pathing.path.pathbuilder.MaxSpeed;
import org.beaverbots.beaver.pathing.path.pathbuilder.PathBuilder;
import org.beaverbots.beaver.pathing.pidf.PIDF;
import org.beaverbots.beaver.pathing.pidf.PIDFAxis;
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
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.function.ToDoubleFunction;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousV0Point5 extends CommandRuntimeOpMode {
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
        intake = new Intake();
        stopper = new Stopper();

        usageRatio = PathBuilder.createHolonomicUsage(1 / Constants.drivetrainPowerConversionFactorX, 1 / Constants.drivetrainPowerConversionFactorY, 1 / Constants.drivetrainPowerConversionFactorTheta);

        register(gamepad, pinpoint, limelight, fusedLocalizer, voltageSensor, shooter, intake, stopper, drivetrain);
        limelight.goalPipeline();

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
        currentPosition = pinpoint.getPosition();

        schedule(
                new Sequential(
                        shootNear(),
                        intakeSpike(2),
                        shootNear(),
                        intakeSpike(2),
                        shootNear(),
                        intakeSpike(2),
                        shootNear(),
                        new Instant(() -> {
                            shooter.spin(0);
                        }),
                        leaveNear()
                )
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", pinpoint.getPosition().toString());
        CrossModeStorage.position = pinpoint.getPosition();
    }

    private PathBuilder newPathBuilder() {
        return new PathBuilder(currentPosition.toList(), mirror, true);
    }

    private void updateCurrentPosition(Path holdPath) {
        currentPosition = new DrivetrainState(holdPath.position(0));
    }

    private Command shootNear() {
        final double X = -16;
        final double Y = 24;
        final double SHOOTER_RPM = 2200;
        final double MAX_ERROR = 50;
        final double HOOD_ANGLE = 0.3;
        final double EASING = 1.8;

        DrivetrainState position = new DrivetrainState(
                X,
                Y,
                Localizer.wind(
                        Math.atan2(
                                Constants.GOAL_Y - Y,
                                Constants.GOAL_X - X
                        ), currentPosition.getTheta()
                )
        );
        double lateralDistance = currentPosition.lateralDistance(position);
        double angularDistance = currentPosition.angularDistance(position);


        Pair<Path, Path> path = newPathBuilder()
                .linearTo(position.toList(), EASING,
                        lateralDistance / Constants.getMaxLateralVelocity() +
                                angularDistance / Constants.getMaxAngularVelocity()
                                + EASING)
                .stop(EASING)
                .build();

        updateCurrentPosition(path.second);

        return new Sequential(
                new Instant(() -> {
                    shooter.spin(SHOOTER_RPM);
                    shooter.setHood(HOOD_ANGLE);
                }),
                followPathTemplate(path.first),
                new RunUntil(
                        new Sequential(
                                new First(
                                        new WaitUntil(() -> shooter.getError() < MAX_ERROR),
                                        new Wait(10)
                                ),
                                new Instant(() -> {
                                    intake.spin(1);
                                    stopper.spin(1);
                                }),
                                new Wait(5)
                        ),
                        followPathTemplate(path.second)
                ),
                new Instant(() -> {
                    intake.spin(0);
                    stopper.spin(0);
                })
        );
    }

    private Command shootFar() {
        final double X = -16;
        final double Y = 24;
        final double SHOOTER_RPM = 3000;
        final double MAX_ERROR = 50;
        final double HOOD_ANGLE = 0.72;
        final double EASING = 1.8;

        DrivetrainState position = new DrivetrainState(
                X,
                Y,
                Localizer.wind(
                        Math.atan2(
                                Constants.GOAL_Y - Y,
                                Constants.GOAL_X - X
                        ), currentPosition.getTheta()
                )
        );
        double lateralDistance = currentPosition.lateralDistance(position);
        double angularDistance = currentPosition.angularDistance(position);


        Pair<Path, Path> path = newPathBuilder()
                .linearTo(position.toList(), EASING,
                        lateralDistance / Constants.getMaxLateralVelocity() +
                                angularDistance / Constants.getMaxAngularVelocity()
                                + EASING)
                .stop(EASING)
                .build();

        updateCurrentPosition(path.second);

        return new Sequential(
                new Instant(() -> {
                    shooter.spin(SHOOTER_RPM);
                    shooter.setHood(HOOD_ANGLE);
                }),
                followPathTemplate(path.first),
                new RunUntil(
                        new Sequential(
                                new First(
                                        new WaitUntil(() -> shooter.getError() < MAX_ERROR),
                                        new Wait(10)
                                ),
                                new Instant(() -> {
                                    intake.spin(0.5);
                                    stopper.spin(1);
                                }),
                                new Wait(5)
                        ),
                        followPathTemplate(path.second)
                ),
                new Instant(() -> {
                    intake.spin(0);
                    stopper.spin(0);
                })
        );
    }

    private Command intakeSpike(int spike) {
        // Using setup manual dimensions (middle of shark fin), rather than CAD.
        final double SPIKE_1_X = -11.78125;
        //final double SPIKE_2_X = 11.78125;
        final double SPIKE_2_X = 7;
        final double SPIKE_3_X = 35.34375;

        final double BEZIER_1_Y = 20;
        final double BEZIER_2_Y = 40;
        final double BEZIER_3_Y = 50;

        final double EASING = 2;

        double spikeX;
        switch (spike) {
            case 1:
                spikeX = SPIKE_1_X;
                break;
            case 2:
                spikeX = SPIKE_2_X;
                break;
            case 3:
                spikeX = SPIKE_3_X;
                break;
            default:
                throw new IllegalArgumentException("Spike must be 1, 2, or 3");
        }

        DrivetrainState position1 = new DrivetrainState(spikeX, BEZIER_1_Y, Math.PI / 2);
        DrivetrainState position2 = new DrivetrainState(spikeX, BEZIER_2_Y, Math.PI / 2);
        DrivetrainState position3 = new DrivetrainState(spikeX, BEZIER_3_Y, Math.PI / 2);
        DrivetrainState position0 = new DrivetrainState(position1.toVector().mapMultiply(2).subtract(position2.toVector()));

        double maxLateralSpeed = Math.max(
                MaxSpeed.cubicBezier(
                        currentPosition.toVector().getSubVector(0, 2),
                        currentPosition.toVector().getSubVector(0, 2),
                        position0.toVector().getSubVector(0, 2),
                        position1.toVector().getSubVector(0, 2)
                ),
                MaxSpeed.cubicBezier(
                        position1.toVector().getSubVector(0, 2),
                        position2.toVector().getSubVector(0, 2),
                        position3.toVector().getSubVector(0, 2),
                        position3.toVector().getSubVector(0, 2)
                )
        );

        double maxAngularSpeed = Math.max(
                MaxSpeed.cubicBezier(
                        currentPosition.toVector().getSubVector(2, 1),
                        currentPosition.toVector().getSubVector(2, 1),
                        position0.toVector().getSubVector(2, 1),
                        position1.toVector().getSubVector(2, 1)
                ),
                MaxSpeed.cubicBezier(
                        position1.toVector().getSubVector(2, 1),
                        position2.toVector().getSubVector(2, 1),
                        position3.toVector().getSubVector(2, 1),
                        position3.toVector().getSubVector(2, 1)
                )
        );


        Pair<Path, Path> path = newPathBuilder()
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), EASING,

                        maxLateralSpeed / Constants.getMaxLateralVelocity() +
                                maxAngularSpeed / Constants.getMaxAngularVelocity()
                                + EASING
                )
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), EASING,

                        maxLateralSpeed / Constants.getMaxLateralVelocity() +
                                maxAngularSpeed / Constants.getMaxAngularVelocity()
                                + EASING
                )
                .stop(EASING).build();

        updateCurrentPosition(path.second);

        return new Sequential(
                new Instant(() -> {
                    intake.spin(1);
                    stopper.spin(-1);
                }),
                followPathTemplate(path.first),
                new Instant(() -> {
                    intake.spin(0);
                    stopper.spin(0);
                })
        );
    }

    private Command leaveNear() {
        final double X = -47.675857;
        final double Y = 23.531253;
        final double EASING = 0.6;

        DrivetrainState position = new DrivetrainState(X, Y, currentPosition.getTheta());
        double distance = currentPosition.lateralDistance(position);

        Pair<Path, Path> path = newPathBuilder()
                .linearTo(position.toList(), EASING, distance / Constants.getMaxLateralVelocity() + EASING)
                .stop(EASING, EASING)
                .build();

        updateCurrentPosition(path.second);

        return new Sequential(
                followPathTemplate(path.first),
                followPathTemplate(path.second)
        );
    }

    private Command leaveFar() {
        final double X = 60;
        final double Y = 34;
        final double EASING = 0.6;

        DrivetrainState position = new DrivetrainState(X, Y, currentPosition.getTheta());
        double distance = currentPosition.lateralDistance(position);

        Pair<Path, Path> path = newPathBuilder()
                .linearTo(position.toList(), EASING, distance / Constants.getMaxLateralVelocity() + EASING)
                .stop(EASING, EASING)
                .build();

        updateCurrentPosition(path.second);

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
