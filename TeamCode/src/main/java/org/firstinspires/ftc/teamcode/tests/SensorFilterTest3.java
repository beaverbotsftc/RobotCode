package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.random.Well19937c;
import org.beaverbots.beaver.SensorFusion;

import java.util.Locale;

@Autonomous(group = "Tests")
public class SensorFilterTest3 extends LinearOpMode {
    // Simulation parameters
    private static final double SIMULATION_DURATION_S = 10.0;
    private static final int LOOP_HZ = 50;
    private static final double DT = 1.0 / LOOP_HZ;

    // UKF Tuning
    private static final double UKF_ALPHA = 0.1;

    // Random number generator for noise simulation
    private final RandomGenerator rng = new Well19937c();

    private static class TestResult {
        public final double positionRmse;
        public final double velocityRmse;

        public TestResult(double posRmse, double velRmse) {
            this.positionRmse = posRmse;
            this.velocityRmse = velRmse;
        }
    }

    @Override
    public void runOpMode() {
        waitForStart();

        RobotLog.i("--- Starting UKF Stress Test ---");

        // --- Test 1: Easy Case (Baseline) ---
        TestResult result1 = runLinearTest(
                "Test 1: Easy Case",
                new ArrayRealVector(new double[]{0.0, 2.5}),    // True initial state [pos, vel]
                new ArrayRealVector(new double[]{0.0, 0.0}),    // Initial guess
                MatrixUtils.createRealDiagonalMatrix(new double[]{1.0, 1.0}), // Initial uncertainty
                MatrixUtils.createRealDiagonalMatrix(new double[]{0.01, 0.1}),// Process noise
                MatrixUtils.createRealDiagonalMatrix(new double[]{0.25})      // Sensor noise
        );
        logResult(result1, "Test 1: Easy Case");


        // --- Test 2: High Sensor Noise ---
        TestResult result2 = runLinearTest(
                "Test 2: High Sensor Noise",
                new ArrayRealVector(new double[]{0.0, 2.5}),
                new ArrayRealVector(new double[]{0.0, 0.0}),
                MatrixUtils.createRealDiagonalMatrix(new double[]{1.0, 1.0}),
                MatrixUtils.createRealDiagonalMatrix(new double[]{0.01, 0.1}),
                MatrixUtils.createRealDiagonalMatrix(new double[]{4.0}) // Much higher sensor noise
        );
        logResult(result2, "Test 2: High Sensor Noise");


        // --- Test 3: Bad Initial Guess ---
        TestResult result3 = runLinearTest(
                "Test 3: Bad Initial Guess",
                new ArrayRealVector(new double[]{0.0, 2.5}),
                new ArrayRealVector(new double[]{-10.0, 20.0}), // Very wrong initial guess
                MatrixUtils.createRealDiagonalMatrix(new double[]{100.0, 100.0}), // High initial uncertainty
                MatrixUtils.createRealDiagonalMatrix(new double[]{0.01, 0.1}),
                MatrixUtils.createRealDiagonalMatrix(new double[]{0.25})
        );
        logResult(result3, "Test 3: Bad Initial Guess");

        // --- Test 4: Non-Linear Motion (Turning Robot) ---
        runNonLinearTest();


        RobotLog.i("--- UKF Stress Test Finished ---");
    }

    private TestResult runLinearTest(String testName, RealVector trueInitialState, RealVector initialGuess, RealMatrix initialCovariance, RealMatrix processNoise, RealMatrix sensorCovariance) {
        RobotLog.i("Running: %s", testName);

        // State: [position, velocity]
        SensorFusion.PredictorFunction predictor = (state, control, dt) ->
                new ArrayRealVector(new double[]{state.getEntry(0) + state.getEntry(1) * dt, state.getEntry(1)});

        SensorFusion.MeasurementFunction measurementFunc = (state) ->
                new ArrayRealVector(new double[]{state.getEntry(0)});

        SensorFusion ukf = new SensorFusion(2, initialGuess, initialCovariance, UKF_ALPHA, predictor, processNoise);

        RealVector trueState = trueInitialState.copy();
        double sumPosSquaredError = 0;
        double sumVelSquaredError = 0;
        int steps = 0;

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SIMULATION_DURATION_S) {
            // Predict
            ukf.predict(new ArrayRealVector(0), DT);

            // Simulate
            trueState = predictor.apply(trueState, null, DT);
            double measurementNoise = rng.nextGaussian() * Math.sqrt(sensorCovariance.getEntry(0, 0));
            RealVector measurement = new ArrayRealVector(new double[]{trueState.getEntry(0) + measurementNoise});

            // Update
            ukf.update(measurement, sensorCovariance, measurementFunc);

            // Accumulate error
            sumPosSquaredError += Math.pow(trueState.getEntry(0) - ukf.getMean().getEntry(0), 2);
            sumVelSquaredError += Math.pow(trueState.getEntry(1) - ukf.getMean().getEntry(1), 2);
            steps++;

            sleep((long) (DT * 1000));
        }

        return new TestResult(Math.sqrt(sumPosSquaredError / steps), Math.sqrt(sumVelSquaredError / steps));
    }


    private void runNonLinearTest() {
        RobotLog.i("Running: Test 4: Non-Linear Motion");

        // State: [x, y, theta]. Control: [v, omega]
        SensorFusion.PredictorFunction predictor = (state, control, dt) -> {
            double x = state.getEntry(0);
            double y = state.getEntry(1);
            double theta = state.getEntry(2);
            double v = control.getEntry(0);
            double omega = control.getEntry(1);

            double newX = x + v * Math.cos(theta) * dt;
            double newY = y + v * Math.sin(theta) * dt;
            double newTheta = theta + omega * dt;
            return new ArrayRealVector(new double[]{newX, newY, newTheta});
        };

        // We only measure x and y position
        SensorFusion.MeasurementFunction measurementFunc = (state) ->
                new ArrayRealVector(new double[]{state.getEntry(0), state.getEntry(1)});

        RealMatrix processNoise = MatrixUtils.createRealDiagonalMatrix(new double[]{0.01, 0.01, 0.001});
        RealMatrix sensorCovariance = MatrixUtils.createRealDiagonalMatrix(new double[]{0.1, 0.1});
        RealMatrix initialCovariance = MatrixUtils.createRealDiagonalMatrix(new double[]{1.0, 1.0, 0.1});
        RealVector initialGuess = new ArrayRealVector(new double[]{0, 0, 0});

        SensorFusion ukf = new SensorFusion(3, initialGuess, initialCovariance, UKF_ALPHA, predictor, processNoise);

        RealVector trueState = new ArrayRealVector(new double[]{0, 0, 0});
        RealVector controlInput = new ArrayRealVector(new double[]{1.0, Math.PI / 4}); // 1 m/s, 45 deg/s turn

        double sumPosSquaredError = 0;
        double sumThetaSquaredError = 0;
        int steps = 0;

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SIMULATION_DURATION_S) {
            // Predict
            ukf.predict(controlInput, DT);

            // Simulate
            trueState = predictor.apply(trueState, controlInput, DT);
            double xNoise = rng.nextGaussian() * Math.sqrt(sensorCovariance.getEntry(0, 0));
            double yNoise = rng.nextGaussian() * Math.sqrt(sensorCovariance.getEntry(1, 1));
            RealVector measurement = new ArrayRealVector(new double[]{trueState.getEntry(0) + xNoise, trueState.getEntry(1) + yNoise});

            // Update
            ukf.update(measurement, sensorCovariance, measurementFunc);

            // Accumulate error
            double dx = trueState.getEntry(0) - ukf.getMean().getEntry(0);
            double dy = trueState.getEntry(1) - ukf.getMean().getEntry(1);
            sumPosSquaredError += dx * dx + dy * dy;
            sumThetaSquaredError += Math.pow(trueState.getEntry(2) - ukf.getMean().getEntry(2), 2);
            steps++;
            sleep((long) (DT * 1000));
        }

        double xyRmse = Math.sqrt(sumPosSquaredError / steps);
        double thetaRmse = Math.sqrt(sumThetaSquaredError / steps);

        RobotLog.i(String.format(Locale.US,
                "Test 4: Non-Linear Motion finished. Positional RMSE: %.4f m, Theta RMSE: %.4f rad",
                xyRmse, thetaRmse
        ));
    }


    private void logResult(TestResult result, String testName) {
        RobotLog.i(String.format(Locale.US,
                "%s finished. Position RMSE: %.4f m, Velocity RMSE: %.4f m/s",
                testName, result.positionRmse, result.velocityRmse
        ));
    }
}