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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(group = "Tests")
public class SensorFilterTest1 extends LinearOpMode {
    // Simulation parameters
    private static final double SIMULATION_DURATION_S = 12.0;
    private static final int LOOP_DELAY_MS = 20; // Run at ~50Hz

    // System dynamics
    private static final int STATE_DIMENSIONALITY = 2; // [position, velocity]
    private static final int MEASUREMENT_DIMENSIONALITY = 1; // [position]
    private static final double TRUE_INITIAL_POSITION = 0.0;
    private static final double TRUE_VELOCITY = 2.5; // m/s

    // UKF Tuning
    private static final double UKF_ALPHA = 0.1; // Spread of sigma points

    // Noise parameters (variances)
    // How much we trust our prediction model. Small values = high trust.
    private static final double PROCESS_NOISE_POS = 0.01;
    private static final double PROCESS_NOISE_VEL = 0.05;
    // How much we trust our sensor readings. Small values = high trust.
    private static final double SENSOR_NOISE_POS = 4.5;


    @Override
    public void runOpMode() {
        // --- 1. SETUP ---
        RandomGenerator rng = new Well19937c();

        // State: [position, velocity]
        SensorFusion.PredictorFunction predictor = (state, control, dt) -> {
            double pos = state.getEntry(0);
            double vel = state.getEntry(1);
            // x_new = x_old + v*dt
            // v_new = v_old
            return new ArrayRealVector(new double[]{pos + vel * dt, vel});
        };

        // We only measure position
        SensorFusion.MeasurementFunction measurementFunc = (state) ->
                new ArrayRealVector(new double[]{state.getEntry(0)});

        // Define noise matrices
        RealMatrix processNoise = MatrixUtils.createRealDiagonalMatrix(
                new double[]{PROCESS_NOISE_POS, PROCESS_NOISE_VEL});
        RealMatrix sensorCovariance = MatrixUtils.createRealDiagonalMatrix(
                new double[]{SENSOR_NOISE_POS});

        // Initialize UKF with an initial guess
        // We guess position 0, but velocity 0 (which is wrong) to see it converge.
        RealVector initialMean = new ArrayRealVector(new double[]{0.0, 0.0});
        // High initial uncertainty
        RealMatrix initialCovariance = MatrixUtils.createRealDiagonalMatrix(new double[]{10.0, 10.0});

        SensorFusion ukf = new SensorFusion(
                STATE_DIMENSIONALITY,
                initialMean,
                initialCovariance,
                UKF_ALPHA,
                predictor,
                processNoise);

        // Data storage for plotting
        List<Double> timeData = new ArrayList<>();
        List<Double> truePositionData = new ArrayList<>();
        List<Double> measuredPositionData = new ArrayList<>();
        List<Double> estimatedPositionData = new ArrayList<>();
        List<Double> estimatedVelocityData = new ArrayList<>();
        List<Double> posUncertaintyData = new ArrayList<>(); // Standard deviation

        RobotLog.i("UKF Test: Initialization complete. Waiting for start.");
        waitForStart();

        // --- 2. SIMULATION LOOP ---
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        double truePosition = TRUE_INITIAL_POSITION;

        while (opModeIsActive() && timer.seconds() < SIMULATION_DURATION_S) {
            double dt = loopTimer.seconds();
            if (dt == 0) { // Avoid division by zero on first loop
                loopTimer.reset();
                sleep(LOOP_DELAY_MS);
                continue;
            }
            loopTimer.reset();

            // PREDICT: Move the UKF state forward based on the model
            ukf.predict(new ArrayRealVector(0), dt); // No control input

            // SIMULATE: Update ground truth and generate a noisy measurement
            truePosition += TRUE_VELOCITY * dt;
            double measurementNoise = rng.nextGaussian() * Math.sqrt(SENSOR_NOISE_POS);
            double measuredPosition = truePosition + measurementNoise;
            RealVector measurement = new ArrayRealVector(new double[]{measuredPosition});

            // UPDATE: Correct the UKF state with the new measurement
            ukf.update(measurement, sensorCovariance, measurementFunc);

            // LOG DATA: Store results for plotting
            timeData.add(timer.seconds());
            truePositionData.add(truePosition);
            measuredPositionData.add(measuredPosition);
            estimatedPositionData.add(ukf.getMean().getEntry(0));
            estimatedVelocityData.add(ukf.getMean().getEntry(1));
            // Standard deviation is the square root of variance (the diagonal element)
            posUncertaintyData.add(Math.sqrt(ukf.getCovariance().getEntry(0, 0)));

            sleep(LOOP_DELAY_MS);
        }

        // --- 3. GENERATE PLOT ---
        RobotLog.i("UKF Test: Simulation finished. Generating Python plotting code.");
        generatePythonPlottingCode(timeData, truePositionData, measuredPositionData, estimatedPositionData, estimatedVelocityData, posUncertaintyData);
    }

    private void generatePythonPlottingCode(List<Double> time, List<Double> truePos, List<Double> measuredPos, List<Double> estPos, List<Double> estVel, List<Double> posUnc) {
        StringBuilder sb = new StringBuilder();
        sb.append("import matplotlib.pyplot as plt\n");
        sb.append("import numpy as np\n\n");

        sb.append(String.format("time = %s\n", formatList(time)));
        sb.append(String.format("true_pos = %s\n", formatList(truePos)));
        sb.append(String.format("measured_pos = %s\n", formatList(measuredPos)));
        sb.append(String.format("est_pos = %s\n", formatList(estPos)));
        sb.append(String.format("est_vel = %s\n", formatList(estVel)));
        sb.append(String.format("pos_unc = %s\n\n", formatList(posUnc)));
        sb.append(String.format("true_vel = %.4f\n\n", TRUE_VELOCITY));

        sb.append("est_pos = np.array(est_pos)\n");
        sb.append("pos_unc = np.array(pos_unc)\n\n");

        sb.append("plt.style.use('seaborn-v0_8-whitegrid')\n");
        sb.append("fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)\n\n");

        sb.append("# Position Plot\n");
        sb.append("ax1.plot(time, true_pos, 'g-', linewidth=2, label='Ground Truth Position')\n");
        sb.append("ax1.plot(time, measured_pos, 'r.', markersize=4, alpha=0.6, label='Noisy Measurements')\n");
        sb.append("ax1.plot(time, est_pos, 'b-', linewidth=2, label='UKF Estimated Position')\n");
        sb.append("ax1.fill_between(time, est_pos - 2 * pos_unc, est_pos + 2 * pos_unc, color='blue', alpha=0.2, label='95% Confidence Interval')\n");
        sb.append("ax1.set_title('UKF Performance: Position Estimation')\n");
        sb.append("ax1.set_ylabel('Position (m)')\n");
        sb.append("ax1.legend()\n\n");

        sb.append("# Velocity Plot\n");
        sb.append("ax2.axhline(y=true_vel, color='g', linestyle='-', linewidth=2, label='Ground Truth Velocity')\n");
        sb.append("ax2.plot(time, est_vel, 'b-', linewidth=2, label='UKF Estimated Velocity')\n");
        sb.append("ax2.set_title('UKF Performance: Velocity Estimation')\n");
        sb.append("ax2.set_xlabel('Time (s)')\n");
        sb.append("ax2.set_ylabel('Velocity (m/s)')\n");
        sb.append("ax2.legend()\n\n");

        sb.append("plt.tight_layout()\n");
        sb.append("plt.show()\n");

        logLongString(sb.toString());
    }

    private String formatList(List<Double> list) {
        StringBuilder sb = new StringBuilder();
        sb.append("[");
        for (int i = 0; i < list.size(); i++) {
            sb.append(String.format(Locale.US, "%.4f", list.get(i)));
            if (i < list.size() - 1) {
                sb.append(",");
            }
        }
        sb.append("]");
        return sb.toString();
    }

    private void logLongString(String str) {
        for (String line : str.split("\n")) {
            RobotLog.i(line);
        }
    }
}