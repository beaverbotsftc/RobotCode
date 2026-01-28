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
public class SensorFilterTest2 extends LinearOpMode {
    // Simulation parameters
    private static final double SIMULATION_DURATION_S = 10.0;
    private static final int LOOP_DELAY_MS = 20; // Run at ~50Hz

    // System dynamics
    private static final double TRUE_VELOCITY = 2.5; // m/s

    // --- TUNING DEMONSTRATION PARAMETERS ---
    // The ACTUAL noise of our simulated sensor. We'll make this quite high.
    private static final double ACTUAL_SENSOR_NOISE_STD_DEV = 4.5;

    // What we TELL the filter the noise is. We'll set this to a very small
    // value to make the filter "over-trust" the sensor.
    private static final double BELIEVED_SENSOR_NOISE_STD_DEV = 0.2;
    // ---

    @Override
    public void runOpMode() {
        RandomGenerator rng = new Well19937c();

        // State: [position, velocity]
        SensorFusion.PredictorFunction predictor = (state, control, dt) -> {
            double pos = state.getEntry(0);
            double vel = state.getEntry(1);
            return new ArrayRealVector(new double[]{pos + vel * dt, vel});
        };

        SensorFusion.MeasurementFunction measurementFunc = (state) ->
                new ArrayRealVector(new double[]{state.getEntry(0)});

        // The filter's belief about sensor noise is configured to be extremely low (high trust)
        final double believedSensorVariance = BELIEVED_SENSOR_NOISE_STD_DEV * BELIEVED_SENSOR_NOISE_STD_DEV;
        RealMatrix sensorCovarianceForFilter = MatrixUtils.createRealDiagonalMatrix(
                new double[]{believedSensorVariance});

        // Other UKF parameters
        RealMatrix processNoise = MatrixUtils.createRealDiagonalMatrix(new double[]{0.001, 0.01});
        RealVector initialMean = new ArrayRealVector(new double[]{0.0, TRUE_VELOCITY}); // Start with a good guess
        RealMatrix initialCovariance = MatrixUtils.createRealDiagonalMatrix(new double[]{1.0, 1.0});

        SensorFusion ukf = new SensorFusion(
                2, initialMean, initialCovariance, 0.1, predictor, processNoise);

        // Data storage for plotting
        List<Double> timeData = new ArrayList<>();
        List<Double> truePositionData = new ArrayList<>();
        List<Double> measuredPositionData = new ArrayList<>();
        List<Double> estimatedPositionData = new ArrayList<>();
        List<Double> estimatedVelocityData = new ArrayList<>();
        List<Double> posUncertaintyData = new ArrayList<>();

        RobotLog.i("UKF Tuning Test: Waiting for start. This test will demonstrate over-trusting the sensor.");
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        double truePosition = 0.0;

        while (opModeIsActive() && timer.seconds() < SIMULATION_DURATION_S) {
            double dt = loopTimer.seconds();
            if (dt == 0) {
                loopTimer.reset();
                sleep(LOOP_DELAY_MS);
                continue;
            }
            loopTimer.reset();

            ukf.predict(new ArrayRealVector(0), dt);

            // SIMULATE: Use the ACTUAL high noise value
            truePosition += TRUE_VELOCITY * dt;
            double measurementNoise = rng.nextGaussian() * ACTUAL_SENSOR_NOISE_STD_DEV;
            double measuredPosition = truePosition + measurementNoise;
            RealVector measurement = new ArrayRealVector(new double[]{measuredPosition});

            // UPDATE: The filter uses its incorrect, overly optimistic belief about the sensor noise
            ukf.update(measurement, sensorCovarianceForFilter,
                    new ArrayRealVector(new double[] {0, 0, 0}),
                    measurementFunc);

            // LOG DATA
            timeData.add(timer.seconds());
            truePositionData.add(truePosition);
            measuredPositionData.add(measuredPosition);
            estimatedPositionData.add(ukf.getMean().getEntry(0));
            estimatedVelocityData.add(ukf.getMean().getEntry(1));
            posUncertaintyData.add(Math.sqrt(ukf.getCovariance().getEntry(0, 0)));

            sleep(LOOP_DELAY_MS);
        }

        RobotLog.i("UKF Tuning Test: Simulation finished. Generating Python plotting code.");
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
        sb.append("fig.suptitle('UKF Performance: Over-Trusting Noisy Sensor Data', fontsize=16)\n\n");

        sb.append("# Position Plot: The estimate will be jerky and chase the noisy measurements.\n");
        sb.append("# The confidence interval will be deceptively narrow, showing false confidence.\n");
        sb.append("ax1.plot(time, true_pos, 'g-', linewidth=2, label='Ground Truth Position')\n");
        sb.append("ax1.plot(time, measured_pos, 'r.', markersize=4, alpha=0.5, label='Noisy Measurements')\n");
        sb.append("ax1.plot(time, est_pos, 'b-', linewidth=2, label='UKF Estimated Position (Jerky)')\n");
        sb.append("ax1.fill_between(time, est_pos - 2 * pos_unc, est_pos + 2 * pos_unc, color='blue', alpha=0.2, label='95% Confidence (False)')\n");
        sb.append("ax1.set_title('Position Estimate is Noisy and Overconfident')\n");
        sb.append("ax1.set_ylabel('Position (m)')\n");
        sb.append("ax1.legend()\n\n");

        sb.append("# Velocity Plot: The velocity estimate will be extremely erratic and noisy.\n");
        sb.append("# The filter interprets random sensor noise as real, high-speed movement.\n");
        sb.append("ax2.axhline(y=true_vel, color='g', linestyle='-', linewidth=2, label='Ground Truth Velocity')\n");
        sb.append("ax2.plot(time, est_vel, 'b-', linewidth=2, label='UKF Estimated Velocity (Erratic)')\n");
        sb.append("ax2.set_title('Velocity Estimate is Wildly Unstable')\n");
        sb.append("ax2.set_xlabel('Time (s)')\n");
        sb.append("ax2.set_ylabel('Velocity (m/s)')\n");
        sb.append("ax2.legend()\n\n");

        sb.append("plt.tight_layout(rect=[0, 0, 1, 0.96])\n");
        sb.append("plt.show()\n");

        logLongString(sb.toString());
    }

    private String formatList(List<Double> list) {
        StringBuilder sb = new StringBuilder();
        sb.append("[");
        for (int i = 0; i < list.size(); i++) {
            sb.append(String.format(Locale.US, "%.4f", list.get(i)));
            if (i < list.size() - 1) {
                sb.append(", ");
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