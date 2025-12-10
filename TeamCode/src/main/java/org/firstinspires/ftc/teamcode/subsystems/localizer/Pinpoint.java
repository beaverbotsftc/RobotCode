package org.firstinspires.ftc.teamcode.subsystems.localizer;

import org.beaverbots.BeaverCommand.HardwareManager;
import org.beaverbots.BeaverCommand.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

import java.util.List;

public final class Pinpoint implements Localizer, Subsystem {
    private GoBildaPinpointDriver pinpoint;
    private DrivetrainState currentPose = null;
    private DrivetrainState currentVelocity = null;

    public Pinpoint(DrivetrainState pose) {
        pinpoint = HardwareManager.claim(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(Constants.pinpointXOffset, Constants.pinpointYOffset);
        pinpoint.recalibrateIMU();
        setPosition(pose);
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
        currentPose = new DrivetrainState(pinpoint.getPosition(), pinpoint.getHeading());
        currentVelocity = new DrivetrainState(pinpoint.getVelocity(), pinpoint.getHeadingVelocity());
    }

    public void setPosition(DrivetrainState position) {
        pinpoint.setPosition(position != null ? position.toPose2d() : new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
    }

    public double wind(double theta) {
        return Localizer.wind(theta, getPosition().getTheta());
    }
}