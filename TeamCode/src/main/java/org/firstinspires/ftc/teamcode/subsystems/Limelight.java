package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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
    private enum Pipeline {
        OBELISK,
        GOAL,
    }
    private static int PIPELINE_OBELISK = 0;
    private static int PIPELINE_GOAL = 9;
    private Pipeline currentPipeline;

    private double lastPositionResultTime = Double.NaN;

    private Limelight3A limelight;

    public Limelight() {
        limelight = HardwareManager.claim(Limelight3A.class, "limelight");
        obeliskPipeline();
        limelight.start(); }

    @Override
    public void periodic() {

    }

    public void obeliskPipeline() {
        limelight.pipelineSwitch(PIPELINE_OBELISK);
        currentPipeline = Pipeline.OBELISK;
    }

    public void goalPipeline() {
        limelight.pipelineSwitch(PIPELINE_GOAL);
        currentPipeline = Pipeline.GOAL;
    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    public Motif getMotif(Side side) {
        if (currentPipeline != Pipeline.OBELISK) throw new IllegalStateException("Invalid pipeline currently selected");

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

    public Pair<DrivetrainState, Double> getEstimatedPosition() {
        if (currentPipeline != Pipeline.GOAL) throw new IllegalStateException("Invalid pipeline currently selected");

        // TODO: It isn't synced with Pinpoint yet, so the dt will always be a bit late (compared to what the UKF has, i.e. pinpoint), but whatever.
        long time = System.nanoTime();

        LLResult result = limelight.getLatestResult();
        if (result.getTimestamp() == lastPositionResultTime) return null;
        if (!result.isValid()) return null;

        // Facing in the correct way is about -0.35rad, but it can jitter to like 0.12rad (i.e. facing almost directly towards the camera), so reject those.
        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (Math.abs(fiducial.getTargetPoseRobotSpace().getOrientation().getPitch(AngleUnit.RADIANS)) > 0.1) return null; // The pitch is always 0 (normal parallel to floor)
            if (Math.abs(fiducial.getTargetPoseRobotSpace().getOrientation().getRoll(AngleUnit.RADIANS)) > 0.1) return null; // The roll is always 0 (no tipping robots I hope)
        }

        lastPositionResultTime = result.getTimestamp();

        Position position = result.getBotpose().getPosition();
        YawPitchRollAngles orientation = result.getBotpose().getOrientation();

        double x = DistanceUnit.INCH.fromUnit(position.unit, position.x);
        double y = DistanceUnit.INCH.fromUnit(position.unit, position.y);
        double theta = orientation.getYaw(AngleUnit.RADIANS);

        // Ignores parse latency to avoid double counting it
        return new Pair<>(new DrivetrainState(x, y, theta), (double) (time - result.getControlHubTimeStampNanos()) / 1e9 + result.getCaptureLatency() / 1000 + result.getTargetingLatency() / 1000);
    }
}