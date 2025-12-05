package org.firstinspires.ftc.teamcode.subsystems.localizer;

import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularMatrixException;
import org.beaverbots.BeaverCommand.Subsystem;
import org.beaverbots.BeaverCommand.util.Stopwatch;
import org.beaverbots.BeaverSensor.UnscentedKalmanFilter;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;

import java.util.Arrays;
import java.util.List;
import java.util.Set;

public class FusedLocalizer implements Subsystem, Localizer {
    private Pinpoint pinpoint;
    private Limelight limelight;

    private UnscentedKalmanFilter filter;

    private RealVector lastTickPinpointState;
    private RealVector lastFilterPinpointState;
    private RealVector highFrequencyPose;

    private Stopwatch stopwatch;

    // Measurement Noise Covariance (R) for Limelight
    private final RealMatrix limelightR;

    // Chi-Squared Threshold for 3 Degrees of Freedom (X, Y, Theta)
    // p = 0.05 (95% Confidence) => 7.815
    // p = 0.01 (99% Confidence) => 11.345
    private static final double OUTLIER_THRESHOLD = 7.815;

    public Set<Subsystem> getDependencies() {
        return Set.of(limelight);
    }

    public FusedLocalizer(Pinpoint pinpoint, Limelight limelight, DrivetrainState initialPose) {
        this.pinpoint = pinpoint;
        this.limelight = limelight;

        limelight.goalPipeline();

        RealVector initialPoseVector = new ArrayRealVector(initialPose.toArray());

        lastTickPinpointState = initialPoseVector;
        lastFilterPinpointState = initialPoseVector;
        highFrequencyPose = initialPoseVector.copy();

        this.limelightR = new Array2DRowRealMatrix(new double[][]{
                {3.87499225, 0, 0},
                {0, 3.87499225, 0},
                {0, 0, 3.87499225}
        }).scalarMultiply(3);

        this.filter = new UnscentedKalmanFilter(
                3,
                initialPoseVector,
                new Array2DRowRealMatrix(new double[][]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}),
                0.1,
                (RealVector state, RealVector control, double dt) -> applyPinpointDelta(state, control),
                new Array2DRowRealMatrix(new double[][]{{0.01, 0, 0}, {0, 0.01, 0}, {0, 0, 0.005}})
        );

        stopwatch = new Stopwatch();
    }

    ///  Correct for potential pinpoint reference frame drift
    private RealVector applyPinpointDelta(RealVector currentState, RealVector controlInput) {
        double theta = currentState.getEntry(2);

        double dXPinpoint = controlInput.getEntry(0);
        double dYPinpoint = controlInput.getEntry(1);
        double dThetaPinpoint = controlInput.getEntry(2);
        double thetaPinpoint = controlInput.getEntry(3); // The raw (maybe) bad heading

        double thetaCorrection = theta - thetaPinpoint;

        // "Un-rotate" the Pinpoint delta and "Re-rotate" into Trusted frame
        double cos = Math.cos(thetaCorrection);
        double sin = Math.sin(thetaCorrection);

        double dXGlobal = dXPinpoint * cos - dYPinpoint * sin;
        double dYGlobal = dXPinpoint * sin + dYPinpoint * cos;

        return currentState.add(new ArrayRealVector(new double[]{dXGlobal, dYGlobal, dThetaPinpoint}));
    }

    public void periodic() {
        RealVector currentRawPinpointState = new ArrayRealVector(pinpoint.getPosition().toArray());

        RealVector instantDelta = currentRawPinpointState.subtract(lastTickPinpointState);
        RealVector instantControl = new ArrayRealVector(new double[] {
                instantDelta.getEntry(0),
                instantDelta.getEntry(1),
                instantDelta.getEntry(2),
                currentRawPinpointState.getEntry(2) // Pass current raw heading for correction
        });

        highFrequencyPose = applyPinpointDelta(highFrequencyPose, instantControl);

        lastTickPinpointState = currentRawPinpointState;

        double dt = stopwatch.getDt();

        RealVector cumulativeDelta = currentRawPinpointState.subtract(lastFilterPinpointState);
        RealVector totalControl = new ArrayRealVector(new double[] {
                cumulativeDelta.getEntry(0),
                cumulativeDelta.getEntry(1),
                cumulativeDelta.getEntry(2),
                currentRawPinpointState.getEntry(2)
        });

        filter.predict(totalControl, dt);

        DrivetrainState limelightEstimation = limelight.getEstimatedPosition();

        if (limelightEstimation != null) {
            double woundLimelightTheta = Localizer.wind(limelightEstimation.getTheta(), filter.getMean().getEntry(2));

            RealVector measurementVector = new ArrayRealVector(new double[] {
                    limelightEstimation.getX(),
                    limelightEstimation.getY(),
                    woundLimelightTheta
            });

            if (isMeasurementValid(measurementVector)) {
                RobotLog.dd("FusedLocalizer", "Update Accepted: " + Arrays.toString(measurementVector.toArray()));
                filter.update(measurementVector, limelightR, x -> x);
            } else {
                RobotLog.dd("FusedLocalizer", "Update Rejected (Outlier): " + Arrays.toString(measurementVector.toArray()));
            }
        }

        // 5. Update High Frequency Pose to match Filter Mean (Fusion)
        highFrequencyPose = filter.getMean();
        lastFilterPinpointState = currentRawPinpointState;
    }

    private boolean isMeasurementValid(RealVector z) {
        try {
            RealVector x = filter.getMean();

            // Calculate Innovation: y = z - x
            // (Note: Angle wrapping is handled before creating vector 'z')
            RealVector y = z.subtract(x);

            // Calculate Innovation Covariance: S = P + R
            // P = State Covariance (Uncertainty of where we think we are)
            // R = Measurement Covariance (Uncertainty of the sensor)
            RealMatrix P = filter.getCovariance();
            RealMatrix S = P.add(limelightR);

            // Mahalanobis Distance Squared: d^2 = y^T * S^-1 * y
            // This normalizes the error 'y' by the standard deviations in 'S'
            RealMatrix S_inv = new LUDecomposition(S).getSolver().getInverse();
            double chi2 = S_inv.preMultiply(y).dotProduct(y);

            // If chi2 < 7.815, there is a 95% chance this is a valid measurement
            // If chi2 > 7.815, it is likely a glitch, reflection, or shaken camera
            return chi2 < OUTLIER_THRESHOLD;

        } catch (SingularMatrixException | UnsupportedOperationException e) {
            // I think Pinpoint will just take it from here, as there is noting to get it our of this spiral.
            return false;
        }
    }

    public DrivetrainState getPosition() {
        return new DrivetrainState(highFrequencyPose);
    }

    public DrivetrainState getVelocity() {
        return pinpoint.getVelocity();
    }

    public List<Double> getPositionAsList() {
        double[] meanArray = highFrequencyPose.toArray();
        return List.of(meanArray[0], meanArray[1], meanArray[2]);
    }

    public List<Double> getVelocityAsList() {
        return pinpoint.getVelocityAsList();
    }

    public double wind(double theta) {
        return Localizer.wind(theta, getPosition().getTheta());
    }
}