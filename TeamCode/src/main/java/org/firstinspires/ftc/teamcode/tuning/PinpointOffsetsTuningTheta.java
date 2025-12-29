package org.firstinspires.ftc.teamcode.tuning;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.optimize.BayesianOptimizer;
import org.beaverbots.beaver.optimize.kernels.RBFKernel;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.SimpleControl;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

@Autonomous
public class PinpointOffsetsTuningTheta extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private static BayesianOptimizer optimizer = new BayesianOptimizer(new RBFKernel(), new Pair<>(
            new ArrayRealVector(new double[] {-5, -5}),
            new ArrayRealVector(new double[] {-3.5, -3.5})
    ), 0.9, 10);

    private RealVector point;

    private Pinpoint pinpoint;
    private Drivetrain drivetrain;

    private Gamepad gamepad;

    private Intake intake;
    private Shooter shooter;

    private double loss;

    Stopwatch stopwatch = new Stopwatch();

    private void applyPoint(RealVector point) {
        Constants.pinpointXOffset = point.getEntry(0);
        Constants.pinpointYOffset = point.getEntry(1);
    }

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        point = optimizer.findNextPoint();
        applyPoint(point);

        RobotLog.d(point.toString());
        try {
            RobotLog.d(optimizer.getBestObservedPoint().toString());
        } catch (IllegalStateException ignored) {}

        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        drivetrain = new MecanumDrivetrain(1);

        gamepad = new Gamepad(gamepad1);
    }

    @Override
    public void onStart() {
        register(voltageSensor, pinpoint, drivetrain);

        register(gamepad);

        schedule(new Sequential(
                new WaitUntil(() -> gamepad.getCross()),
                new Instant(() -> pinpoint.setPosition(new DrivetrainState(0, 0, 0))),
                new Instant(() -> stopwatch.reset()),
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, Math.PI / 2))),
                new WaitUntil(() -> pinpoint.getPosition().getTheta() > Math.PI),
                new Instant(() -> RobotLog.d(String.format("Position: %s", pinpoint.getPosition().toString()))),
                new Instant(() -> loss = pinpoint.getPosition().lateralDistance(new DrivetrainState(0, 0, 0))),
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0))),
                new Instant(() -> RobotLog.d(String.format("Loss: %f", loss))),
                new WaitUntil(() -> gamepad.getCircle()),
                new Instant(() -> optimizer.addObservedPoint(point, loss)),
                new Instant(() -> gamepad1.rumble(200)),
                new Instant(() -> requestOpModeStop())
            )
        );
    }
}
