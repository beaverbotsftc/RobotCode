package org.firstinspires.ftc.teamcode.tuning;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.RunUntil;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.optimize.BayesianOptimizer;
import org.beaverbots.beaver.optimize.kernels.ARDRBFKernel;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Stopwatch;

import java.util.List;

@Autonomous
public class CRServoTuning extends CommandRuntimeOpMode {
    private static BayesianOptimizer optimizer = new BayesianOptimizer(new ARDRBFKernel(), new Pair<>(
            new ArrayRealVector(new double[]{0, 0, 0, 0.01, 0}),
            new ArrayRealVector(new double[]{1, 1, 1, 1, 1e3})
    ), 0.9, 20);

    private static final double TIME_PER_TRIAL = 5;

    private CRServo servo;
    private AnalogInput encoder;
    private double loss;

    Stopwatch stopwatch = new Stopwatch();

    PIDF pidf;
    RealVector point;

    double bestLoss = 99999999999999999999999.0;
    RealVector bestPoint = null;

    public void onInit() {
        servo = HardwareManager.claim(CRServo.class, "servo");
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = HardwareManager.claim(AnalogInput.class, "encoder");
    }

    private double getAngle() {
        return encoder.getVoltage() / encoder.getMaxVoltage() * 2 * Math.PI;
    }

    int i = 0;

    Command getTuningScript() {
        i++;
        return new Sequential(
                new Instant(() -> loss = 0),
                new Instant(() -> {
                    point = optimizer.findNextPoint();
                    pidf = new PIDF(List.of(new PIDFAxis(new PIDFAxis.K(
                            point.getEntry(0), point.getEntry(1), point.getEntry(2), 0, 1, 1, point.getEntry(3), point.getEntry(4)
                    ))));
                }),
                new RunUntil(
                        new WaitUntil(() -> Math.abs(getAngle() - 1.5 * Math.PI) < 0.1),
                        new Repeat(() -> servo.setPower(0.5 * (getAngle() - 1.5 * Math.PI)))
                ),
                new Instant(() -> servo.setPower(0)),
                new Instant(() -> stopwatch.reset()),
                new RunUntil(
                        new WaitUntil(() -> stopwatch.getElapsed() > TIME_PER_TRIAL),
                        new Repeat(() -> {
                            double dt = stopwatch.getDt();
                            loss += dt * stopwatch.getElapsed() * Math.abs(getAngle() - Math.PI);
                            servo.setPower(pidf.update(List.of(getAngle() - Math.PI), List.of(0.0), dt).get(0));
                        })
                ),
                new Instant(() -> servo.setPower(0)),
                new Instant(() -> {
                    if (loss < bestLoss) {
                        bestLoss = loss;
                        bestPoint = point;
                    }
                }),
                new Instant(() -> optimizer.addObservedPoint(point, loss)),
                new Instant(() -> schedule(getTuningScript()))
        );
    }

    @Override
    public void onStart() {
        schedule(
                getTuningScript()
        );
    }

    public void periodic() {
        if (bestPoint != null) {
            telemetry.addData("Best Loss", bestLoss);
            telemetry.addData("Best P", bestPoint.getEntry(0));
            telemetry.addData("Best I", bestPoint.getEntry(1));
            telemetry.addData("Best D", bestPoint.getEntry(2));
            telemetry.addData("Best Tau", bestPoint.getEntry(3));
            telemetry.addData("Best Gamma", bestPoint.getEntry(4));
            telemetry.addData("Iteration", i);
        }
    }
}
