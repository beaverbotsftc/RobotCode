package org.firstinspires.ftc.teamcode.tuning;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.premade.Cycle;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.RunUntil;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.optimize.BayesianOptimizer;
import org.beaverbots.beaver.optimize.kernels.ARDRBFKernel;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Stopwatch;

@Autonomous(group = "tuning")
public class SwerveServoTuning2 extends CommandOpMode {
    private static BayesianOptimizer optimizer = new BayesianOptimizer(new ARDRBFKernel(), new Pair<>(
            new ArrayRealVector(new double[]{0.2, 0, 0, 0.01, 0}),
            new ArrayRealVector(new double[]{0.5, 1, 0.5, 1, 1e3})
    ), 0.95, 20);

    private static final double TIME_PER_TRIAL = 1.0;

    private CRServo servo;
    private AnalogInput encoder;
    private double loss;

    private double lastPower = Double.NaN;

    Stopwatch stopwatch = new Stopwatch();

    PIDFAxis pidf;
    RealVector point;
    int runs = 0;

    double bestLoss = 99999999999999999999999.0;
    int bestIter = 0;
    RealVector bestPoint = null;

    public void onInit() {
        servo = HardwareManager.claim(CRServo.class, "back right servo");
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = HardwareManager.claim(AnalogInput.class, "back right servo encoder");
    }

    private double getAngle() {
        return encoder.getVoltage() / encoder.getMaxVoltage() * 2 * Math.PI;
    }

    int i = 0;

    Command getTuningScript() {
        i++;
        return new Sequential(
                new Instant(() -> {
                    loss = 0;
                    lastPower = Double.NaN;
                }),
                new Instant(() -> {
                    point = optimizer.findNextPoint();
                    pidf = new PIDFAxis(new PIDFAxis.K(
                            point.getEntry(0), point.getEntry(1), point.getEntry(2),
                            new double[]{0}, 1, 1, point.getEntry(3), point.getEntry(4), 0.1
                    ));
                }),
                new Cycle(
                        j -> j >= 3,
                        new Instant(() -> pidf.reset()),
                        new RunUntil(
                                new WaitUntil(() -> Math.abs(getAngle() - 1.5 * Math.PI) < 0.1),
                                new Repeat(() -> servo.setPower(4 * (getAngle() - 1.5 * Math.PI)))
                        ),
                        new Instant(() -> {
                            servo.setPower(0);
                            stopwatch.reset();
                            lastPower = Double.NaN; // Clear residual power from the reset phase
                        }),

                        new RunUntil(
                                new WaitUntil(() -> stopwatch.getElapsed() > TIME_PER_TRIAL),
                                new Repeat(() -> {
                                    double dt = stopwatch.getDt();
                                    double elapsed = stopwatch.getElapsed();
                                    double currentAngle = getAngle();
                                    double error = currentAngle - Math.PI;

                                    loss += dt * elapsed * Math.abs(error);

                                    double power = Math.max(-1, Math.min(pidf.update(error, new double[]{0.0}, dt), 1));
                                    if (Double.isNaN(lastPower)) loss -= Math.abs(power) * 2; // The power has to go to 0 eventually, don't penalize it for that.
                                    else loss += Math.abs(power - lastPower) * 2; // Already should take dt into account by getAngle() not changing very much

                                    servo.setPower(power);
                                    lastPower = power;
                                })
                        ),

                        new Instant(() -> {
                            double finalError = Math.abs(getAngle() - Math.PI);

                            // Use a nominal dt of 20ms for the final power calculation
                            double finalPower = Math.max(-1, Math.min(pidf.update(getAngle() - Math.PI, new double[]{0.0}, 0.02), 1));

                            loss += (finalError * finalError) * 20.0;

                            loss += Math.abs(finalPower) * 5; // It got a loss benefit for starting out at high power. Ending with high power to keep it is cheating, so it needs to overpower that.
                        }),
                        new Instant(() -> servo.setPower(0))
                ),

                new Instant(() -> {
                    servo.setPower(0);

                    loss /= 3.0;

                    if (loss < bestLoss) {
                        bestLoss = loss;
                        bestIter = i;
                        bestPoint = point;
                    }

                    optimizer.addObservedPoint(point, loss);

                    schedule(getTuningScript());
                })
        );
    }

    @Override
    public void onStart() {
        // Kick off the first iteration
        schedule(getTuningScript());
    }

    @Override
    public void periodic() {
        if (bestPoint != null) {
            telemetry.addData("--- BEST CONSTANTS ---", "");
            telemetry.addData("Best Iteration", bestIter);
            telemetry.addData("Best Loss", String.format("%.4f", bestLoss));
            telemetry.addData("P", String.format("%.4f", bestPoint.getEntry(0)));
            telemetry.addData("I", String.format("%.4f", bestPoint.getEntry(1)));
            telemetry.addData("D", String.format("%.4f", bestPoint.getEntry(2)));
            telemetry.addData("Tau (D-Filter)", String.format("%.4f", bestPoint.getEntry(3)));
            telemetry.addData("Gamma (I-Zone)", String.format("%.4f", bestPoint.getEntry(4)));
            telemetry.addLine();
            telemetry.addData("Current Iteration", i);
            telemetry.addData("Current Angle", String.format("%.3f", getAngle()));
            telemetry.addData("Current Point", point);
            telemetry.update();
        }
    }
}