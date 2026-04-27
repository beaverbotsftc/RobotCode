package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.premade.Cycle;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.RunUntil;
import org.beaverbots.beaver.command.premade.Wait;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.util.Pair;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.ArrayList;

@Autonomous(group = "tuning")
public class AllTheTuningButForTheta extends CommandOpMode {
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

    private Stopwatch stopwatch;
    private ArrayList<Pair<Double, Double>> points;

    public void onStart() {
        schedule(new Cycle(
                i -> false,
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0.001))),
                new WaitUntil(() -> gamepad.getX()),
                new Instant(() -> points = new ArrayList<>()),
                new Instant(() -> localizer.setPosition(new Transform(0, 0, 0))),
                new Wait(1),
                new Instant(() -> stopwatch = new Stopwatch()),
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0.5))),
                new RunUntil(
                        new Wait(2),
                        new Repeat(() -> points.add(new Pair<>(stopwatch.getElapsed(), localizer.getPosition().getTheta())))
                ),
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0))),
                new Repeat(() -> addLine(points.toString()))
        ));
    }
}
