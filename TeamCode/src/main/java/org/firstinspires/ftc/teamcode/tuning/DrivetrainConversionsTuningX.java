package org.firstinspires.ftc.teamcode.tuning;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Sequential;
import org.beaverbots.BeaverCommand.util.Stopwatch;
import org.beaverbots.BeaverCommand.util.WaitUntil;
import org.beaverbots.BeaverOptimize.BayesianOptimizer;
import org.beaverbots.BeaverOptimize.RBFKernel;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.DrivetrainState;
import org.firstinspires.ftc.teamcode.commands.DirectControl;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

import java.util.Timer;

@TeleOp
public class DrivetrainConversionsTuning extends CommandRuntimeOpMode {
    private static BayesianOptimizer optimizer = new BayesianOptimizer(new RBFKernel(), new Pair<>(
            new ArrayRealVector(new double[] {0.01, 0.01, 0.1}),
            new ArrayRealVector(new double[] {0.02, 0.02, 0.2})
    ), 0.9);

    private RealVector point;

    private Pinpoint pinpoint;
    private Drivetrain drivetrain;

    private Gamepad gamepad;

    private Intake intake;
    private Shooter shooter;

    private double loss = 0;

    Stopwatch stopwatch = new Stopwatch();

    private void applyPoint(RealVector point) {
        Constants.drivetrainPowerConversionFactorX = point.getEntry(0);
        Constants.drivetrainPowerConversionFactorY = point.getEntry(1);
        Constants.drivetrainPowerConversionFactorTheta = point.getEntry(2);
    }

    @Override
    public void onInit() {
        point = optimizer.findNextPoint();
        applyPoint(point);

        RobotLog.d(point.toString());
        try {
            RobotLog.d(optimizer.getBestObservedPoint().toString());
        } catch (IllegalStateException ignored) {}

        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        drivetrain = new MecanumDrivetrain(1);

        gamepad = new Gamepad(gamepad1);

        intake = new Intake();
        shooter = new Shooter();
    }

    @Override
    public void onStart() {
        register(pinpoint, drivetrain);

        register(gamepad);

        register(intake, shooter);

        schedule(new Sequential(
                new WaitUntil(() -> gamepad.getCross()),
                new Instant(() -> pinpoint.resetPosition(new DrivetrainState(0, 0, 0))),
                new Instant(() -> stopwatch.reset()),
                new Instant(() -> drivetrain.move(new DrivetrainState(24, 0, 0))),
                new WaitUntil(() -> stopwatch.getElapsed() > 2),
                new Instant(() -> RobotLog.d(String.format("X: %f", pinpoint.getPosition().getX()))),
                new Instant(() -> loss += Math.pow((pinpoint.getPosition().getX() - 24 * 2), 2)),
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0))),
                new WaitUntil(() -> gamepad.getCross()),
                new Instant(() -> pinpoint.resetPosition(new DrivetrainState(0, 0, 0))),
                new Instant(() -> stopwatch.reset()),
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 24, 0))),
                new WaitUntil(() -> stopwatch.getElapsed() > 2),
                new Instant(() -> RobotLog.d(String.format("Y: %f", pinpoint.getPosition().getY()))),
                new Instant(() -> loss += Math.pow((pinpoint.getPosition().getY() - 24 * 2), 2)),
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0))),
                new WaitUntil(() -> gamepad.getCross()),
                new Instant(() -> pinpoint.resetPosition(new DrivetrainState(0, 0, 0))),
                new Instant(() -> stopwatch.reset()),
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, Math.PI))),
                new WaitUntil(() -> stopwatch.getElapsed() > 2),
                new Instant(() -> RobotLog.d(String.format("Theta: %f", pinpoint.getPosition().getTheta()))),
                new Instant(() -> loss += Math.pow((pinpoint.getPosition().getTheta() - Math.PI * 2), 2) * 10),
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0))),
                new Instant(() -> RobotLog.d(String.format("Loss: %f", loss))),
                new WaitUntil(() -> gamepad.getCircle()),
                new Instant(() -> optimizer.addObservedPoint(point, loss)),
                new DirectControl(gamepad, drivetrain, intake, shooter)
            )
        );
    }
}
