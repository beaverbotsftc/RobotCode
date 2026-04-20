
package org.firstinspires.ftc.teamcode.subsystems.localizer;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.beaverbots.beaver.util.Transform;

import java.util.List;

public final class Pinpoint implements Localizer, Subsystem {

    private GoBildaPinpointDriver pinpoint;
    private Transform currentPose = new Transform(0, 0, 0);
    private Transform currentVelocity = new Transform(0, 0, 0);

    public Pinpoint(Transform pose) {
        pinpoint = HardwareManager.claim(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(Constants.pinpointXOffset, Constants.pinpointYOffset, DistanceUnit.MM);
        pinpoint.setBulkReadScope(
                GoBildaPinpointDriver.Register.DEVICE_STATUS,
                GoBildaPinpointDriver.Register.X_POSITION,
                GoBildaPinpointDriver.Register.Y_POSITION,
                GoBildaPinpointDriver.Register.H_ORIENTATION,
                GoBildaPinpointDriver.Register.X_VELOCITY,
                GoBildaPinpointDriver.Register.Y_VELOCITY,
                GoBildaPinpointDriver.Register.H_VELOCITY
        );
        pinpoint.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        pinpoint.recalibrateIMU();
        setPosition(pose);
    }

    public List<Double> getPositionAsList() {
        return List.of(currentPose.getX(), currentPose.getY(), currentPose.getTheta());
    }

    public List<Double> getVelocityAsList() {
        return List.of(currentVelocity.getX(), currentVelocity.getY(), currentVelocity.getTheta());
    }

    public Transform getPosition() {
        return currentPose;
    }

    public Transform getVelocity() {
        return currentVelocity;
    }

    public void periodic() {
        pinpoint.update();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            RobotLog.ee("BeaverBots", String.format("Pinpoint fault: %s", pinpoint.getDeviceStatus()));
        }

        currentPose = new Transform(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS));

        currentVelocity = new Transform(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }

    public void setPosition(Transform position) {
        pinpoint.setPosition(position != null ? position.toPose2d() : new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        currentPose = position;
    }
}