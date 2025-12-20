package org.firstinspires.ftc.teamcode.tuning;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.RunUntil;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.Wait;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.optimize.BayesianOptimizer;
import org.beaverbots.beaver.optimize.kernels.RBFKernel;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

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
