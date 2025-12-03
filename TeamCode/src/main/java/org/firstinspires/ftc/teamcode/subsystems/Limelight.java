package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.HardwareManager;
import org.beaverbots.BeaverCommand.Subsystem;
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

    private double lastPositionAttemptTime = Double.NaN;

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

    public void goalPipeline() {
        limelight.pipelineSwitch(PIPELINE_GOAL);
        currentPipeline = Pipeline.GOAL;
    }

    public LLStatus status() {
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
                        return Motif.GREEN_PURPLE_PURPLE;
                    case 22:
                        return Motif.PURPLE_GREEN_PURPLE;
                    case 23:
                        return Motif.PURPLE_PURPLE_GREEN;
                }
            case 2:
                Set<Integer> ids = Set.of(resultsList.get(0).getFiducialId(), resultsList.get(1).getFiducialId());
                switch (side) {
                    case RED:
                        if (ids.equals(Set.of(21, 22))) return Motif.GREEN_PURPLE_PURPLE;
                        else if (ids.equals(Set.of(22, 23))) return Motif.PURPLE_GREEN_PURPLE;
                        else return Motif.PURPLE_PURPLE_GREEN;
                    case BLUE:
                        if (ids.equals(Set.of(21, 23))) return Motif.GREEN_PURPLE_PURPLE;
                        else if (ids.equals(Set.of(21, 22))) return Motif.PURPLE_GREEN_PURPLE;
                        else return Motif.PURPLE_PURPLE_GREEN;
                }
        }
        return null;
    }

    public DrivetrainState getEstimatedPosition() {
        if (currentPipeline != Pipeline.GOAL) throw new IllegalStateException("Invalid pipeline currently selected");

        LLResult result = limelight.getLatestResult();
        if (result.getTimestamp() == lastPositionAttemptTime) return null;
        if (!result.isValid()) return null;

        Position position = result.getBotpose().getPosition();
        YawPitchRollAngles orientation = result.getBotpose().getOrientation();

        double x = -DistanceUnit.INCH.fromUnit(position.unit, position.x) + 72;
        double y = -DistanceUnit.INCH.fromUnit(position.unit, position.y) + 72;
        double theta = -orientation.getYaw(AngleUnit.RADIANS) - Math.PI / 4;

        return new DrivetrainState(x, y, theta);
    }
}