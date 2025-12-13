package org.firstinspires.ftc.teamcode.tuning;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Repeat;
import org.beaverbots.BeaverCommand.util.RunUntil;
import org.beaverbots.BeaverCommand.util.Sequential;
import org.beaverbots.BeaverCommand.util.Wait;
import org.beaverbots.BeaverCommand.util.WaitUntil;
import org.beaverbots.BeaverOptimize.BayesianOptimizer;
import org.beaverbots.BeaverOptimize.util.RBFKernel;
import org.beaverbots.beavertracking.HolonomicFollowPath;
import org.beaverbots.beavertracking.PIDF;
import org.beaverbots.beavertracking.PIDFAxis;
import org.beaverbots.beavertracking.Path;
import org.beaverbots.beavertracking.PathAxis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.SimpleControl;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.List;

@Autonomous
public class ShooterConversionFactor extends CommandRuntimeOpMode {
    private static BayesianOptimizer optimizer = new BayesianOptimizer(new RBFKernel(), new Pair<>(
            new ArrayRealVector(new double[]{ Constants.shooterFrictionConversionFactor * 0.8 }),
            new ArrayRealVector(new double[]{ Constants.shooterFrictionConversionFactor })
    ), 0.8, 5);

    private RealVector point;

    private VoltageSensor voltageSensor;
    private Shooter shooter;
    private Gamepad gamepad;

    private static final double targetRpm = 3000;

    private double loss;

    private void applyPoint(RealVector point) {
        Constants.shooterFrictionConversionFactor = point.getEntry(0);
    }

    @Override
    public void onInit() {
        point = optimizer.findNextPoint();
        applyPoint(point);

        RobotLog.d(point.toString());
        try {
            RobotLog.d(optimizer.getBestObservedPoint().toString());
        } catch (IllegalStateException ignored) {}

        voltageSensor = new VoltageSensor();
        shooter = new Shooter(voltageSensor);
        gamepad = new Gamepad(gamepad1);
    }

    @Override
    public void onStart() {
        register(voltageSensor, shooter, gamepad);

        schedule(new Sequential(
                        new WaitUntil(() -> gamepad.getCross()),
                        new Instant(() -> shooter.spin(targetRpm)),
                        new RunUntil(
                                new Wait(12),
                                new Repeat(() -> telemetry.addData("RPM:", shooter.getVelocity()))
                        ),
                        new Instant(() -> loss = Math.pow(shooter.getVelocity() - targetRpm, 2)),
                        new Instant(() -> shooter.spin(0)),
                        new Instant(() -> RobotLog.d(String.format("Loss: %f", loss))),
                        new WaitUntil(() -> gamepad.getCircle()),
                        new Instant(() -> optimizer.addObservedPoint(point, loss)),
                        new Instant(() -> gamepad.rumble(1000)),
                        new Wait(1)
                )
        );
    }
}
