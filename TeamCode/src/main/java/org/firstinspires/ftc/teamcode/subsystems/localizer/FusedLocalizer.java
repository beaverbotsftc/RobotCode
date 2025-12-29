package org.firstinspires.ftc.teamcode.subsystems.localizer;

import android.util.Pair;

import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.SensorFusion;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;

import java.util.List;
import java.util.Set;

public class FusedLocalizer implements Subsystem, Localizer {
    private Localizer localizer;
    private Limelight limelight;

    private SensorFusion filter;

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

        lastFilterPinpointState = initialPoseVector;
        highFrequencyPose = initialPoseVector.copy();

        this.filter = new SensorFusion(
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

        DrivetrainState deltaPinpoint = new DrivetrainState(
                controlInput.getEntry(0),
                controlInput.getEntry(1),
                controlInput.getEntry(2)
        );
        double thetaPinpoint = controlInput.getEntry(3);

        DrivetrainState deltaTrusted = deltaPinpoint.rotate(theta - thetaPinpoint);

        return currentState.add(deltaTrusted.toVector());
    }

    public void periodic() {
        RealVector currentRawPinpointState = new ArrayRealVector(localizer.getPosition().toArray());
        double dt = stopwatch.getDt();

        RealVector cumulativeDelta = currentRawPinpointState.subtract(lastFilterPinpointState);
        RealVector totalControl = new ArrayRealVector(new double[]{
                cumulativeDelta.getEntry(0),
                cumulativeDelta.getEntry(1),
                cumulativeDelta.getEntry(2),
                currentRawPinpointState.getEntry(2)
        });

        filter.predict(totalControl, dt);

        lastFilterPinpointState = currentRawPinpointState;

        Pair<DrivetrainState, Double> limelightEstimation = limelight.getEstimatedPosition();

        if (limelightEstimation != null) {
            RealVector measurement = new ArrayRealVector(new double[]{
                    limelightEstimation.first.getX(),
                    limelightEstimation.first.getY(),
                    wind(limelightEstimation.first.getTheta())
            }).add(getVelocity().toVector().mapMultiply(limelightEstimation.second));

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

    public List<Double> getPositionAsList() {
        double[] meanArray = highFrequencyPose.toArray();
        return List.of(meanArray[0], meanArray[1], meanArray[2]);
    }

    public DrivetrainState getVelocity() {
        return localizer.getVelocity().rotate(highFrequencyPose.getEntry(2) - localizer.getPosition().getTheta());
    }

    public List<Double> getVelocityAsList() {
        return getVelocity().toList();
    }

    public RealMatrix getCovariance() {
        return filter.getCovariance();
    }

    public double wind(double theta) {
        return Localizer.wind(theta, getPosition().getTheta());
    }
}