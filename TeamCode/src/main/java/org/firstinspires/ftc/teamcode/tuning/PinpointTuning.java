package org.firstinspires.ftc.teamcode.tuning;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Sequential;
import org.beaverbots.BeaverCommand.util.WaitUntil;
import org.beaverbots.BeaverOptimize.BayesianOptimizer;
import org.beaverbots.BeaverOptimize.util.RBFKernel;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class PinpointTuning extends CommandRuntimeOpMode {
    private static BayesianOptimizer optimizer = new BayesianOptimizer(new RBFKernel(), new Pair<>(
            new ArrayRealVector(new double[] {-3.5, -2}),
            new ArrayRealVector(new double[] {-4.5, -3})
    ), 0.9, 3);

    private RealVector point;

    private Pinpoint pinpoint;

    private Gamepad gamepad;

    private void applyPoint(RealVector point) {
        Constants.pinpointXOffset = point.getEntry(0);
        Constants.pinpointYOffset = point.getEntry(1);
    }

    private double getLoss() {
        double x = pinpoint.getPosition().getX();
        double y = pinpoint.getPosition().getY();
        double theta = pinpoint.getPosition().getTheta();
        RobotLog.d(String.valueOf(x));
        RobotLog.d(String.valueOf(y));
        RobotLog.d(String.valueOf(theta));
        return Math.pow(x, 2)
                    + Math.pow(y, 2)
                    + Math.pow(theta, 2);
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

        gamepad = new Gamepad(gamepad1);
    }

    @Override
    public void onStart() {
        register(pinpoint);

        register(gamepad);

        schedule(new Sequential(
                new WaitUntil(() -> gamepad.getCross()),
                new Instant(() -> RobotLog.d(String.valueOf(getLoss()))),
                new WaitUntil(() -> gamepad.getCircle()),
                new Instant(() -> optimizer.addObservedPoint(point, getLoss())),
                new Instant(() -> requestOpModeStop())
            )
        );
    }
}
