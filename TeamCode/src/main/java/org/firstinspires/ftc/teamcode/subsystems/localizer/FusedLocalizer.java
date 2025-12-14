package org.firstinspires.ftc.teamcode.subsystems.localizer;

import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.BeaverCommand.Subsystem;
import org.beaverbots.BeaverCommand.util.Stopwatch;
import org.beaverbots.BeaverSensor.UnscentedKalmanFilter;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;

import java.util.List;
import java.util.Set;

public class FusedLocalizer implements Subsystem, Localizer {
    private Localizer localizer;
    private Limelight limelight;

    private UnscentedKalmanFilter filter;

    private RealVector lastTickPinpointState;
    private RealVector lastFilterPinpointState;
    private RealVector highFrequencyPose;

    private Stopwatch stopwatch;

    public Set<Subsystem> getDependencies() {
        return Set.of(limelight);
    }

    public FusedLocalizer(Localizer localizer, Limelight limelight, DrivetrainState initialPose) {
        this.localizer = localizer;
        this.limelight = limelight;

        limelight.goalPipeline();

        RealVector initialPoseVector = new ArrayRealVector(initialPose.toArray());

        lastTickPinpointState = initialPoseVector;
        lastFilterPinpointState = initialPoseVector;
        highFrequencyPose = initialPoseVector.copy();

        this.filter = new UnscentedKalmanFilter(
                3,
                initialPoseVector,
                new Array2DRowRealMatrix(new double[][]{{144 * 144, 0, 0}, {0, 144 * 144, 0}, {0, 0, Math.PI * Math.PI}}),
                0.01,
                // Now we just reference the helper method here
                (RealVector state, RealVector control, double dt) -> applyPinpointDelta(state, control),
                new Array2DRowRealMatrix(new double[][]{{0.001, 0, 0}, {0, 0.001, 0}, {0, 0, 0.0005}})
        );

        stopwatch = new Stopwatch();
    }

    ///  Correct for potential pinpoint reference frame drift
    private RealVector applyPinpointDelta(RealVector currentState, RealVector controlInput) {
        double theta = currentState.getEntry(2);

        double dXPinpoint = controlInput.getEntry(0);
        double dYPinpoint = controlInput.getEntry(1);
        double dThetaPinpoint = controlInput.getEntry(2);
        double thetaPinpoint = controlInput.getEntry(3); // The raw bad heading

        // Calculate how far off the Pinpoint frame is from our Trusted frame
        double thetaCorrection = theta - thetaPinpoint;

        // "Un-rotate" the Pinpoint delta and "Re-rotate" into Trusted frame
        double cos = Math.cos(thetaCorrection);
        double sin = Math.sin(thetaCorrection);

        double dXGlobal = dXPinpoint * cos - dYPinpoint * sin;
        double dYGlobal = dXPinpoint * sin + dYPinpoint * cos;

        // Return new state (adding the deltas)
        return currentState.add(new ArrayRealVector(new double[]{dXGlobal, dYGlobal, dThetaPinpoint}));
    }

    public void periodic() {
        RealVector currentRawPinpointState = new ArrayRealVector(localizer.getPosition().toArray());
        double dt = stopwatch.getDt();

        RealVector cumulativeDelta = currentRawPinpointState.subtract(lastFilterPinpointState);
        RealVector totalControl = new ArrayRealVector(new double[] {
                cumulativeDelta.getEntry(0),
                cumulativeDelta.getEntry(1),
                cumulativeDelta.getEntry(2),
                currentRawPinpointState.getEntry(2)
        });

        filter.predict(totalControl, dt);

        lastFilterPinpointState = currentRawPinpointState;

        DrivetrainState limelightEstimation = limelight.getEstimatedPosition();

        if (limelightEstimation != null) {
            RealVector measurement = new ArrayRealVector(new double[] {
                    limelightEstimation.getX(),
                    limelightEstimation.getY(),
                    wind(limelightEstimation.getTheta())
            });

            RealMatrix sensorCovariance = new Array2DRowRealMatrix(new double[][]{
                    {3.875, 0, 0},
                    {0, 3.875, 0},
                    {0, 0, 20}
            }).scalarMultiply(3);

            if (filter.isMeasurementInlier(measurement, sensorCovariance, x -> x, 0.05)) {
                filter.update(measurement, sensorCovariance, x -> x);
                RobotLog.i("Measurement accepted");
            } else {
                RobotLog.w("Measurement rejected");
            }
        }

        highFrequencyPose = filter.getMean();
    }

    public DrivetrainState getPosition() {
        return new DrivetrainState(highFrequencyPose);
    }

    public DrivetrainState getVelocity() {
        return localizer.getVelocity();
    }

    public List<Double> getPositionAsList() {
        double[] meanArray = highFrequencyPose.toArray();
        return List.of(meanArray[0], meanArray[1], meanArray[2]);
    }

    public List<Double> getVelocityAsList() {
        return localizer.getVelocityAsList();
    }

    public RealMatrix getCovariance() {
        return filter.getCovariance();
    }

    public double wind(double theta) {
        return Localizer.wind(theta, getPosition().getTheta());
    }
}