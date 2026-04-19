package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.premade.Cycle;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.RunUntil;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.Wait;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.PathAxis;
import org.beaverbots.beaver.pathing.trackers.HolonomicPathTracker;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.List;
import java.util.function.DoubleUnaryOperator;

@Autonomous(group = "tuning")
public class VelocityTuningX extends CommandOpMode {
    private Drivetrain drivetrain;
    private Pinpoint localizer;
    private GamepadEx gamepad;

    public void onInit() {
        VoltageSensor voltageSensor = new VoltageSensor();
        drivetrain = new MecanumDrivetrain(voltageSensor);
        localizer = new Pinpoint(new Transform(0, 0, 0));
        gamepad = new GamepadEx(gamepad1);

        register(drivetrain, gamepad);
    }

    private Transform endPosition = null;
    private double error = 0;
    public static final DoubleUnaryOperator path = t -> t;
    public static final double ACCELERATION_TIME = 1;
    public static final double TIME = 3;
    private Stopwatch stopwatch;

    public void onStart() {
        schedule(new Cycle(
                i -> false,
                new WaitUntil(() -> gamepad.getX()),
                new Instant(() -> localizer.setPosition(new Transform(0, 0, 0))),
                new Wait(1),
                new Instant(() -> stopwatch = new Stopwatch()),
                new Instant(() -> error = 0),
                new RunUntil(
                        new HolonomicFollowPath(
                                new HolonomicPathTracker(
                                        new Path(
                                                List.of(
                                                        new PathAxis(path, 0, TIME),
                                                        new PathAxis(t -> 0, 0, TIME),
                                                        new PathAxis(t -> 0, 0, TIME)
                                                ),
                                                t -> t > TIME
                                        ),
                                        new PIDF(
                                                List.of(
                                                        new PIDFAxis(
                                                                new PIDFAxis.K(
                                                                        0,
                                                                        0,
                                                                        0,
                                                                        new double[]{Constants.pidFVelocityX, Constants.pidFAccelerationX},
                                                                        0,
                                                                        1,
                                                                        0,
                                                                        0
                                                                )
                                                        ),
                                                        new PIDFAxis(
                                                                new PIDFAxis.K(
                                                                        0,
                                                                        0,
                                                                        0,
                                                                        new double[]{Constants.pidFVelocityX, Constants.pidFAccelerationX},
                                                                        0,
                                                                        1,
                                                                        0,
                                                                        0
                                                                )
                                                        ),
                                                        new PIDFAxis(
                                                                new PIDFAxis.K(
                                                                        Constants.pidPHeadingEnforcement,
                                                                        Constants.pidIHeadingEnforcement,
                                                                        Constants.pidDHeadingEnforcement,
                                                                        new double[]{Constants.pidFVelocityX, Constants.pidFAccelerationX},
                                                                        0,
                                                                        1,
                                                                        Constants.pidTauHeadingEnforcement,
                                                                        Constants.pidGammaHeadingEnforcement
                                                                )
                                                        )
                                                )
                                        )
                                ),
                                localizer,
                                drivetrain
                        ),
                        new Sequential(
                                new Wait(ACCELERATION_TIME),
                                new Repeat(() -> error += stopwatch.getDt() * Math.abs(path.applyAsDouble(stopwatch.getElapsed()) - localizer.getPosition().getX()))
                        )
                ),
                new Instant(() -> endPosition = localizer.getPosition()),
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0))),
                new RunUntil(new WaitUntil(() -> gamepad.getX()), new Repeat(() -> {
                    telemetry.addData("Velocity", Constants.pidFVelocityX);
                    telemetry.addData("Acceleration", Constants.pidFAccelerationX);
                    telemetry.addData("End position", endPosition);
                    telemetry.addData("Error", error);
                    Constants.pidFVelocityX += gamepad.getLeftX() * 0.001;
                    Constants.pidFAccelerationX += gamepad.getRightX() * 0.001;

                    drivetrain.move(new Transform(gamepad.getLeftY(), gamepad.getLeftX(), gamepad.getRightX()));
                })),
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0)))
        ));
    }
}
