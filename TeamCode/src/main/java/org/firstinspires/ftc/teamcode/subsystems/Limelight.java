package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;

import java.util.List;
import java.util.Set;

public class Limelight implements Subsystem {
    private static final double M_TO_IN = 39.37007874;

    public enum Pipeline {
        OBELISK,
        LOCALIZATION_GOAL,
    }

    public static class LimelightLocalization {
        private DrivetrainState state;
        private DrivetrainState variance;

        public LimelightLocalization(DrivetrainState state, DrivetrainState variance) {
            this.state = state;
            this.variance = variance;
        }

        public DrivetrainState getState() {
            return state;
        }

        public DrivetrainState getVariance() {
            return variance;
        }

        public String toString() {
            return "State: " + state + "; Stddev: " + variance;
        }
    }

    private static int PIPELINE_OBELISK = 0;
    private static int PIPELINE_LOCALIZATION_GOAL = 9;
    private Pipeline currentPipeline;

    private double lastPositionResultTime = Double.NaN;

    private Limelight3A limelight;

    public Limelight() {
        limelight = HardwareManager.claim(Limelight3A.class, "limelight");
        obeliskPipeline();
        limelight.start();
    }

    @Override
    public void periodic() {

    }

    public void obeliskPipeline() {
        limelight.pipelineSwitch(PIPELINE_OBELISK);
        currentPipeline = Pipeline.OBELISK;
    }

    public void localizationPipeline() {
        limelight.pipelineSwitch(PIPELINE_LOCALIZATION_GOAL);
        currentPipeline = Pipeline.LOCALIZATION_GOAL;
    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    public Pipeline getCurrentPipeline() {
        return currentPipeline;
    }

    public Motif getMotif(Side side) {
        if (currentPipeline != Pipeline.OBELISK)
            throw new IllegalStateException("Invalid pipeline currently selected");

        LLResult results = limelight.getLatestResult();
        if (!results.isValid()) return null;

        // Already filtered by limelight itself
        List<LLResultTypes.FiducialResult> resultsList = results.getFiducialResults();
        switch (resultsList.size()) {
            case 1:
                switch (resultsList.get(0).getFiducialId()) {
                    case 21:
                        return Motif.GPP;
                    case 22:
                        return Motif.PGP;
                    case 23:
                        return Motif.PPG;
                    default:
                        return null;
                }
            case 2:
                Set<Integer> ids = Set.of(resultsList.get(0).getFiducialId(), resultsList.get(1).getFiducialId());
                switch (side) {
                    case RED:
                        if (ids.equals(Set.of(21, 22))) return Motif.GPP;
                        else if (ids.equals(Set.of(22, 23))) return Motif.PGP;
                        else if (ids.equals(Set.of(21, 23))) return Motif.PPG;
                    case BLUE:
                        if (ids.equals(Set.of(21, 23))) return Motif.GPP;
                        else if (ids.equals(Set.of(21, 22))) return Motif.PGP;
                        else if (ids.equals(Set.of(22, 23))) return Motif.PPG;
                    default:
                        return null;
                }
        }
        return null;
    }

    public Pair<LimelightLocalization, Double> getEstimatedPosition() {
        if (currentPipeline != Pipeline.LOCALIZATION_GOAL)
            throw new IllegalStateException("Invalid pipeline currently selected");

        // TODO: It isn't synced with Pinpoint yet, so the dt will always be a bit too long (compared to what the UKF has, i.e. pinpoint), but whatever.
        // TODO: The timestamp returned by limelight for Control Hub time in nanoseconds is wrong, so I'll use milliseconds.
        long time = System.currentTimeMillis();

        LLResult result = limelight.getLatestResult();
        if (result.getTimestamp() == lastPositionResultTime) return null;
        if (!result.isValid()) return null;

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            // TODO: Somehow limelight (or the FTC SDK) thinks that pitch is yaw and yaw is pitch!!! Potential bug in the SDK, idk though.
            if (Math.abs(fiducial.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.RADIANS)) > 0.4)
                return null; // The pitch is always 0 (normal parallel to floor), but because it is mounted like it is, a higher tolerance is required
            if (Math.abs(fiducial.getTargetPoseRobotSpace().getOrientation().getRoll(AngleUnit.RADIANS)) > 0.2)
                return null; // The roll is always 0 (no tipping robots I hope)
        }

        lastPositionResultTime = result.getTimestamp();

        Position position = result.getBotpose().getPosition();
        YawPitchRollAngles orientation = result.getBotpose().getOrientation();

        double x = DistanceUnit.INCH.fromUnit(position.unit, position.x);
        double y = DistanceUnit.INCH.fromUnit(position.unit, position.y);
        double theta = orientation.getYaw(AngleUnit.RADIANS);

        double xVariance = Math.pow(result.getStddevMt1()[0] * M_TO_IN, 2);
        double yVariance = Math.pow(result.getStddevMt1()[1] * M_TO_IN, 2);
        double thetaVariance = Math.pow(Math.toRadians(result.getStddevMt1()[5]), 2);
        RobotLog.d(String.valueOf(thetaVariance));

        // Ignores parse latency to avoid double counting it
        return new Pair<>(new LimelightLocalization(
                new DrivetrainState(x, y, theta),
                new DrivetrainState(xVariance, yVariance, thetaVariance)
        ), (double) (time - result.getControlHubTimeStamp()) / 1000 + result.getCaptureLatency() / 1000 * 0 + result.getTargetingLatency() / 1000 * 0);
    }
}