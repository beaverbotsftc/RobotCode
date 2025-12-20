package org.firstinspires.ftc.teamcode.tests;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.beaver.optimize.BayesianOptimizer;
import org.beaverbots.beaver.optimize.kernels.RBFKernel;

import java.util.function.ToDoubleFunction;

@Autonomous(group = "Tests")
public class OptimizationTest1 extends LinearOpMode {
    final static int MAX_ITERATIONS = 24;
    @Override
    public void runOpMode() {
        BayesianOptimizer optimizer1 = new BayesianOptimizer(new RBFKernel(), new Pair<>(
                new ArrayRealVector(new double[]{ 0 }),
                new ArrayRealVector(new double[]{ 1.5 })),
                0.9, 5);
        // Minimum at f(0.5) = 0
        ToDoubleFunction<RealVector> f1 = (RealVector v) -> {
            double x = v.getEntry(0);
            return (x - 0.5) * (x - 0.5);
        };

        BayesianOptimizer optimizer2 = new BayesianOptimizer(new RBFKernel(), new Pair<>(
                new ArrayRealVector(new double[]{ 0 }),
                new ArrayRealVector(new double[]{ 1 })),
                0.9, 5);
        // Minimum at f(0) = 0.25
        ToDoubleFunction<RealVector> f2 = (RealVector v) -> {
            double x = v.getEntry(0);
            return (x + 0.5) * (x + 0.5);
        };

        BayesianOptimizer optimizer3 = new BayesianOptimizer(new RBFKernel(), new Pair<>(
                new ArrayRealVector(new double[]{ 0 }),
                new ArrayRealVector(new double[]{ 100 })),
                0.99, 5);
        // Minimum at f(12.0936) = -4.8765
        ToDoubleFunction<RealVector> f3 = (RealVector v) -> {
            double x = v.getEntry(0);
            return -(Math.exp(-Math.pow(x - 10, 2)) + 3 * Math.exp(-Math.pow(x - 12, 2)) + 4 * Math.exp(-Math.pow(x - 90, 2)) + 2 * Math.exp(-Math.pow(x - 50, 2)) + Math.sin(10 * x) + Math.cos(x));
        };

        BayesianOptimizer optimizer4 = new BayesianOptimizer(new RBFKernel(), new Pair<>(
                new ArrayRealVector(new double[]{ 0, 0 }),
                new ArrayRealVector(new double[]{ 100, 100 })),
                0.99, 5);
        // Minimum at f(13.6692, 22.3828) = -95.3058
        ToDoubleFunction<RealVector> f4 = (RealVector v) -> {
            double x = v.getEntry(0);
            double y = v.getEntry(1);
            return -(
                    50 * Math.pow(Math.sin(Math.sqrt(x * y) / 10.0), 2)
                    + 40 * Math.exp(-Math.pow((x - y) / 10.0, 2))
                    + Math.pow(Math.sin(x + y), 2)
                    - 20 * Math.pow(Math.tan(x / 40.0), 2)
                    + 30 * Math.exp(-10 * Math.pow(Math.sin((x - 2 * y) / 10.0), 2))
            );
        };

        BayesianOptimizer optimizer5 = new BayesianOptimizer(new RBFKernel(), new Pair<>(
                new ArrayRealVector(new double[]{ -15, 2500, 1920, 0 }),
                new ArrayRealVector(new double[]{ 14, 5000, 1930, 12 })),
                0.99, 5);
        // Minimum at f(10.0002, 4502.0934, 1928.4998, 8.9913) = -400.88513
        ToDoubleFunction<RealVector> f5 = (RealVector v) -> {
            double x = v.getEntry(0);
            double y = v.getEntry(1);
            double z = v.getEntry(2);
            double w = v.getEntry(3);
            return -(
                    250.0 * Math.exp(
                            -(Math.pow(x / 10.0, 2) +
                                    Math.pow((y - 3000.0) / 1000.0, 2) +
                                    Math.pow((z - 1922.0) / 5.0, 2) +
                                    Math.pow((w - 5.0) / 5.0, 2))
                    ) +

                            400.0 * Math.exp(
                                    -(Math.pow((x - 10.0) / 3.0, 2) +
                                            Math.pow((y - 4500.0) / 500.0, 2) +
                                            Math.pow((z - 1928.5) / 0.2, 2) +
                                            Math.pow((w - 9.0) / 3.0, 2))
                            ) +

                            10.0 * Math.cos(2.0 * x) *
                                    Math.cos(y / 20.0) *
                                    Math.cos(5.0 * (z - 1920.0)) *
                                    Math.cos(3.0 * w)
                    );
        };

        RobotLog.i("Starting optimization.");

        waitForStart();

        for (int i = 0; i <= MAX_ITERATIONS; i++) {
            RealVector nextPoint = optimizer1.findNextPoint();
            double value = f1.applyAsDouble(nextPoint);
            optimizer1.addObservedPoint(nextPoint, value);
            RobotLog.i("Test 1: iteration " + i + " point tested " + nextPoint + " : " + value);
            RobotLog.i("Test 1: iteration " + i + " best point " + optimizer1.getBestObservedPoint().first + " : " + optimizer1.getBestObservedPoint().second);
        }

        for (int i = 0; i <= MAX_ITERATIONS; i++) {
            RealVector nextPoint = optimizer2.findNextPoint(); double value = f2.applyAsDouble(nextPoint);
            optimizer2.addObservedPoint(nextPoint, value);
            RobotLog.i("Test 2: iteration " + i + " point tested " + nextPoint + " : " + value);
            RobotLog.i("Test 2: iteration " + i + " best point " + optimizer2.getBestObservedPoint().first + " : " + optimizer2.getBestObservedPoint().second);
        }

        for (int i = 0; i <= MAX_ITERATIONS; i++) {
            RealVector nextPoint = optimizer3.findNextPoint();
            double value = f3.applyAsDouble(nextPoint);
            optimizer3.addObservedPoint(nextPoint, value);
            RobotLog.i("Test 3: iteration " + i + " point tested " + nextPoint + " : " + value);
            RobotLog.i("Test 3: iteration " + i + " best point " + optimizer3.getBestObservedPoint().first + " : " + optimizer3.getBestObservedPoint().second);
        }

        for (int i = 0; i <= MAX_ITERATIONS; i++) {
            RealVector nextPoint = optimizer4.findNextPoint();
            double value = f4.applyAsDouble(nextPoint);
            optimizer4.addObservedPoint(nextPoint, value);
            RobotLog.i("Test 4: iteration " + i + " point tested " + nextPoint + " : " + value);
            RobotLog.i("Test 4: iteration " + i + " best point " + optimizer4.getBestObservedPoint().first + " : " + optimizer4.getBestObservedPoint().second);
        }

        for (int i = 0; i <= MAX_ITERATIONS; i++) {
            RealVector nextPoint = optimizer5.findNextPoint();
            double value = f5.applyAsDouble(nextPoint);
            optimizer5.addObservedPoint(nextPoint, value);
            RobotLog.i("Test 5: iteration " + i + " point tested " + nextPoint + " : " + value);
            RobotLog.i("Test 5: iteration " + i + " best point " + optimizer5.getBestObservedPoint().first + " : " + optimizer5.getBestObservedPoint().second);
        }
    }
}
