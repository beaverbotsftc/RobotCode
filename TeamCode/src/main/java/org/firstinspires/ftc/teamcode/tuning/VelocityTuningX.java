package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.premade.Cycle;
import org.beaverbots.beaver.command.premade.Defer;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Parallel;
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
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDrivetrain;
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
        drivetrain = new SwerveDrivetrain(voltageSensor, false);
        localizer = new Pinpoint(new Transform(0, 0, 0));
        gamepad = new GamepadEx(gamepad1);

        register(localizer, drivetrain, voltageSensor, gamepad);
    }

    private Transform endPosition = null;
    private double error = 0;
    private double error2 = 0;
    public static final DoubleUnaryOperator path = t -> 32 * t;
    public static final double ACCELERATION_TIME = 1;
    public static final double TIME = 1;
    private Stopwatch stopwatch;

    public void onStart() {
        schedule(new Cycle(
                i -> false,
                new WaitUntil(() -> gamepad.getX()),
                new Instant(() -> localizer.setPosition(new Transform(0, 0, 0))),
                new Wait(1),
                new Instant(() -> stopwatch = new Stopwatch()),
                new Instant(() -> error = 0),
                new Instant(() -> error2 = 0),
                new RunUntil(
                        new Defer(() -> new HolonomicFollowPath(
                                new HolonomicPathTracker(
                                        new Path(
                                                List.of(
                                                        new PathAxis(path, 0, TIME + ACCELERATION_TIME),
                                                        new PathAxis(t -> 0, 0, TIME + ACCELERATION_TIME),
                                                        new PathAxis(t -> 0, 0, TIME + ACCELERATION_TIME)
                                                ),
                                                t -> t > (TIME + ACCELERATION_TIME)
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
                                                                        0,
                                                                        0.1
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
                                                                        0,
                                                                        0.1
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
                                                                        0,
                                                                        0.1
                                                                )
                                                        )
                                                )
                                        )
                                ),
                                localizer,
                                drivetrain
                        )
                        ),
                        new Sequential(
                                new Wait(ACCELERATION_TIME),
                                new Repeat(() -> {
                                    double dt = stopwatch.getDt();
                                    error += dt * Math.abs(localizer.getVelocity().getX() - 32);
                                    error2 += dt * (localizer.getVelocity().getX() - 32);
                                })
                        )
                ),
                new Instant(() -> endPosition = localizer.getPosition()),
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0))),
                new RunUntil(new WaitUntil(() -> gamepad.getX()), new Repeat(() -> {
                    telemetry.addData("Velocity", Constants.pidFVelocityX);
                    telemetry.addData("Acceleration", Constants.pidFAccelerationX);
                    telemetry.addData("End position", endPosition);
                    telemetry.addData("Error", error);
                    telemetry.addData("Error2", error2);
                    Constants.pidFVelocityX += (gamepad.getRightTrigger() - gamepad.getLeftTrigger()) * 0.001;
                    //Constants.pidFAccelerationX += gamepad.getRightX() * 0.001;

                    drivetrain.move(new Transform(gamepad.getLeftY(), -gamepad.getLeftX(), -gamepad.getRightX()));
                })),
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0)))
        ));
    }
}
