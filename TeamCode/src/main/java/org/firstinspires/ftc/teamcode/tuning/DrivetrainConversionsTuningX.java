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
import org.beaverbots.BeaverOptimize.util.RBFKernel;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.commands.SimpleControl;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@TeleOp
public class DrivetrainConversionsTuningX extends CommandRuntimeOpMode {
    private static BayesianOptimizer optimizer = new BayesianOptimizer(new RBFKernel(), new Pair<>(
            new ArrayRealVector(new double[] {Constants.drivetrainPowerConversionFactorX}),
            new ArrayRealVector(new double[] {Constants.drivetrainPowerConversionFactorX * 1.2})
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
        Constants.drivetrainPowerConversionFactorX = point.getEntry(0);
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
                    new Instant(() -> stopwatch.reset()),
                    new Instant(() -> drivetrain.move(new DrivetrainState(24, 0, 0))),
                    new WaitUntil(() -> stopwatch.getElapsed() > 2),
                    new Instant(() -> RobotLog.d(String.format("X: %f", pinpoint.getVelocity().getX()))),
                    new Instant(() -> loss = Math.pow((pinpoint.getVelocity().getX() - 24), 2)),
                    new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0))),
                    new Instant(() -> RobotLog.d(String.format("Loss: %f", loss))),
                    new WaitUntil(() -> gamepad.getCircle()),
                    new Instant(() -> optimizer.addObservedPoint(point, loss)),
                    new Instant(() -> gamepad1.rumble(1000)),
                    new SimpleControl(gamepad, drivetrain, intake, shooter)
                )
        );
    }
}
