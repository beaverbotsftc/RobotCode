package org.firstinspires.ftc.teamcode.tests;

import org.beaverbots.BeaverCommand.HardwareManager;
import org.beaverbots.BeaverCommand.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

public final class Pinpoint implements Subsystem {
    private GoBildaPinpointDriver pinpoint;
    private Pose2D currentPose = null;
    private Pose2D currentPoseVelocity = null;

    public Pinpoint(Pose2D initalPose) {
        pinpoint = HardwareManager.claim("pinpoint");
        pinpoint.recalibrateIMU();
        resetPosition(initalPose);
    }

    public void resetPosition(Pose2D pose) {
        pinpoint.setPosition(pose);
    }

    public Pose2D getPosition() {
        return currentPose;
    }

    public Pose2D getVelocity() {
        return currentPoseVelocity;
    }

    public void periodic() {
        pinpoint.update();
        currentPose = pinpoint.getPosition();
        currentPoseVelocity = pinpoint.getVelocity();
    }
}
