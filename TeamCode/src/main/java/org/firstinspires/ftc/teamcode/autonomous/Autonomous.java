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
import java.util.function.DoubleUnaryOperator;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends CommandRuntimeOpMode {
    private static final double SPIKE_1_X = 82.25;
    private static final double SPIKE_2_X = 58.625;
    private static final double SPIKE_3_X = 35;

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

    @Override
    public void onInit() {
        CrossModeStorage.side = side;
        mirror =
                side == Side.RED
                        ? List.of(x -> x, y -> y, theta -> theta)
                        : List.of(x -> x, y -> 144 - y, theta -> -theta);

        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        limelight = new Limelight();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new DrivetrainState(0, 0, 0));
        voltageSensor = new VoltageSensor();
        shooter = new Shooter(voltageSensor);
        intake = new Intake();
        stopper = new Stopper();

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
                        intakeSpike(1),
                        shootNear(),
                        intakeSpike(2),
                        shootNear(),
                        intakeSpike(3),
                        shootNear(),
                        new Instant(() -> {
                            shooter.release();
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
        DrivetrainState position = new DrivetrainState(93.67, 51.38, -0.74);
        double lateralDistance = currentPosition.lateralDistance(position);
        double angularDistance = currentPosition.angularDistance(position);
        double easing = 1.8;

        double shooterRpm = 2200;
        double error = 50;
        double hood = 0.1;

        Pair<Path, Path> path = newPathBuilder()
                .linearTo(position.toList(), easing, Math.max(
                        lateralDistance / Constants.getMaxLateralVelocity(),
                        angularDistance / Constants.getMaxAngularVelocity()
                ) + easing)
                .stop(easing)
                .build();

        updateCurrentPosition(path.second);

        return new Sequential(
                new Instant(() -> {
                    shooter.spin(shooterRpm);
                    shooter.setHood(hood);
                }),
                followPathTemplate(path.first),
                new RunUntil(
                        new Sequential(
                                new First(
                                        new WaitUntil(() -> shooter.getError() < error),
                                        new Wait(10)
                                ),
                                new Instant(() -> {
                                    intake.spin(1);
                                    stopper.spin(1);
                                }),
                                new Wait(2)
                        ),
                        new Instant(() -> {
                            intake.spin(0);
                            stopper.spin(0);
                        }),
                        followPathTemplate(path.second)
                )
        );
    }

    private Command intakeSpike(int spike) {
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

        DrivetrainState position1 = new DrivetrainState(spikeX, 47, -Math.PI / 2);
        DrivetrainState position2 = new DrivetrainState(spikeX, 37.5, -Math.PI / 2);
        DrivetrainState position3 = new DrivetrainState(spikeX, 28, -Math.PI / 2);
        DrivetrainState position0 = new DrivetrainState(position1.toVector().mapMultiply(2).subtract(position2.toVector()));
        double lateralDistance1 = currentPosition.lateralDistance(position0);
        double angularDistance1 = currentPosition.angularDistance(position0);
        double lateralDistance2 = position0.lateralDistance(position1);
        double angularDistance2 = position0.angularDistance(position1);
        double lateralDistance3 = position1.lateralDistance(position2);
        double angularDistance3 = position1.angularDistance(position2);
        double lateralDistance4 = position2.lateralDistance(position3);
        double angularDistance4 = position2.angularDistance(position3);

        // "Convex hull" property; source: AI
        double maxLateralSpeed = 3 * Math.max(lateralDistance1, Math.max(lateralDistance2, Math.max(lateralDistance3, lateralDistance4)));

        double maxAngularSpeed = 3 * Math.max(angularDistance1, Math.max(angularDistance2, Math.max(angularDistance3, angularDistance4)));

        double easing = 0.2;

        Pair<Path, Path> path = newPathBuilder()
                .bezierTo(currentPosition.toList(), position0.toList(), position1.toList(), easing,
                        3
                )
                .bezierTo(position2.toList(), position3.toList(), position3.toList(), easing,
                        3
                )
                .stop(easing)
                .build();

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
        DrivetrainState position = new DrivetrainState(120, 50, currentPosition.getTheta());
        double distance = currentPosition.lateralDistance(position);
        double easing = 0.6;

        Pair<Path, Path> path = newPathBuilder()
                .linearTo(position.toList(), easing, distance / Constants.getMaxLateralVelocity() + easing)
                .stop(easing, easing)
                .build();

        updateCurrentPosition(path.second);

        return new Sequential(
                followPathTemplate(path.first),
                followPathTemplate(path.second)
        );
    }

    private Command leaveFar() {
        DrivetrainState position = new DrivetrainState(50, 50, currentPosition.getTheta());
        double distance = currentPosition.lateralDistance(position);
        double easing = 0.6;

        Pair<Path, Path> path = newPathBuilder()
                .linearTo(position.toList(), easing, distance / Constants.getMaxLateralVelocity() + easing)
                .stop(easing, easing)
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
