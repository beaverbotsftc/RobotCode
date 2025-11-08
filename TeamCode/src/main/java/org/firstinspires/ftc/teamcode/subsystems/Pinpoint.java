package org.firstinspires.ftc.teamcode.subsystems;

import org.beaverbots.BeaverCommand.HardwareManager;
import org.beaverbots.BeaverCommand.Subsystem;
import org.beaverbots.beavertracking.Localizer;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.DrivetrainState;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

import java.util.List;

public final class Pinpoint implements Localizer, Subsystem {
    private GoBildaPinpointDriver pinpoint;
    private DrivetrainState currentPose = null;
    private DrivetrainState currentVelocity = null;

    public Pinpoint(DrivetrainState pose) {
        pinpoint = HardwareManager.claim(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(Constants.pinpointXOffset, Constants.pinpointYOffset);
        pinpoint.recalibrateIMU();
        resetPosition(pose);
    }

    public void resetPosition(DrivetrainState pose) {
        pinpoint.setPosition(pose.toPose2d());
    }

    public List<Double> getPositionAsList() {
        return List.of(currentPose.getX(), currentPose.getY(), currentPose.getTheta());
    }

    public List<Double> getVelocityAsList() {
        return List.of(currentVelocity.getX(), currentVelocity.getY(), currentVelocity.getTheta());
    }

    public DrivetrainState getPosition() {
        return currentPose;
    }

    public DrivetrainState getVelocity() {
        return currentVelocity;
    }

      public void periodic() {
        pinpoint.update();
        currentPose = new DrivetrainState(pinpoint.getPosition());
        currentVelocity = new DrivetrainState(pinpoint.getVelocity());
    }
}