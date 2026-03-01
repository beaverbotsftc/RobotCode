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
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Transform;

import java.util.List;
import java.util.Set;

public class FusedLocalizer implements Subsystem, Localizer {
    private Localizer localizer;
    private Limelight limelight;

    private SensorFusion filter;

    private RealVector lastFilterPinpointState;
    private RealVector highFrequencyPose;

    private Stopwatch stopwatch;

    private RealMatrix initialCovariance;

    private boolean allowLimelight = true;

    public Set<Subsystem> getDependencies() {
        return Set.of(limelight);
    }


    public FusedLocalizer(Localizer localizer, Limelight limelight, Transform initialPose) {
        this(localizer, limelight, initialPose, new Array2DRowRealMatrix(new double[][]{{1e9, 0, 0}, {0, 1e9, 0}, {0, 0, 1e6}}));
    }

    public void resetCovariance() {
        filter.setCovariance(initialCovariance);
    }

    double maxDt = 0;

    public FusedLocalizer(Localizer localizer, Limelight limelight, Transform initialPose, RealMatrix covariance) {
        this.initialCovariance = covariance.copy();
        this.localizer = localizer;
        this.limelight = limelight;

        limelight.localizationPipeline();

        RealVector initialPoseVector = new ArrayRealVector(initialPose.toArray());

        lastFilterPinpointState = initialPoseVector;
        highFrequencyPose = initialPoseVector.copy();

        this.filter = new SensorFusion(
                3,
                initialPoseVector,
                covariance,
                0.05,
                // Now we just reference the helper method here
                (RealVector state, RealVector control, double dt) -> applyPinpointDelta(state, control),
                new Array2DRowRealMatrix(new double[][]{
                        {Constants.lateralVariancePinpoint, 0, 0},
                        {0, Constants.lateralVariancePinpoint, 0},
                        {0, 0, Constants.angularVariancePinpoint}
                })
        );

        stopwatch = new Stopwatch();
    }

    public void enableLimelight() {
        allowLimelight = true;
    }

    public void disableLimelight() {
        allowLimelight = false;
    }

    ///  Correct for potential pinpoint reference frame drift
    private RealVector applyPinpointDelta(RealVector currentState, RealVector controlInput) {
        double theta = currentState.getEntry(2);

        Transform deltaPinpoint = new Transform(
                controlInput.getEntry(0),
                controlInput.getEntry(1),
                controlInput.getEntry(2)
        );
        double thetaPinpoint = controlInput.getEntry(3);

        Transform deltaTrusted = deltaPinpoint.rotate(theta - thetaPinpoint);

        return currentState.add(deltaTrusted.toVector());
    }

    public void periodic() {
        RealVector currentRawPinpointState = new ArrayRealVector(localizer.getPosition().toArray());
        double dt = stopwatch.getDt();
        if (dt > maxDt) {
            maxDt = dt;
        }
        //RobotLog.dd("FusedLocalizer", "maxDt=%.6f", maxDt);

        RealVector cumulativeDelta = currentRawPinpointState.subtract(lastFilterPinpointState);
        RealVector totalControl = new ArrayRealVector(new double[]{
                cumulativeDelta.getEntry(0),
                cumulativeDelta.getEntry(1),
                cumulativeDelta.getEntry(2),
                lastFilterPinpointState.getEntry(2)
                //currentRawPinpointState.getEntry(2)
        });

        filter.predict(totalControl, dt);

        lastFilterPinpointState = currentRawPinpointState;

        if (allowLimelight && limelight.getCurrentPipeline() == Limelight.Pipeline.LOCALIZATION_GOAL) {
            Pair<Limelight.LimelightLocalization, Double> limelightEstimation = limelight.getEstimatedPosition();

            if (limelightEstimation != null && (
                    getVelocity().lateralDistance(new Transform(0, 0, 0)) < 0.5 &&
                    Math.abs(getVelocity().getTheta()) < 0.05
            )
            ) {
                RealVector measurement = new ArrayRealVector(new double[]{
                        limelightEstimation.first.getState().getX(),
                        limelightEstimation.first.getState().getY(),
                        wind(limelightEstimation.first.getState().getTheta())
                }).add(getVelocity().toVector().mapMultiply(limelightEstimation.second));

                RealMatrix sensorCovariance = new Array2DRowRealMatrix(new double[][]{
                        {limelightEstimation.first.getVariance().getX(), 0, 0},
                        {0, limelightEstimation.first.getVariance().getY(), 0},
                        {0, 0, 4 * limelightEstimation.first.getVariance().getTheta()}
                }).scalarMultiply(10);

                //if (filter.isMeasurementInlier(measurement, sensorCovariance, x -> x, 0.05)) {
                filter.update(measurement, sensorCovariance, new ArrayRealVector(new double[] { Constants.minLateralVarianceLimelight, Constants.minLateralVarianceLimelight, Constants.minAngularVariancePinpoint }), x -> x);
                //}
            }
        }

        highFrequencyPose = filter.getMean();
    }

    public Transform getPosition() {
        return new Transform(highFrequencyPose);
    }

    public List<Double> getPositionAsList() {
        double[] meanArray = highFrequencyPose.toArray();
        return List.of(meanArray[0], meanArray[1], meanArray[2]);
    }

    public Transform getVelocity() {
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