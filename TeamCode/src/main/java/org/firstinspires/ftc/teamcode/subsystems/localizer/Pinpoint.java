
package org.firstinspires.ftc.teamcode.subsystems.localizer;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

import java.util.List;

public final class Pinpoint implements Localizer, Subsystem {

    private GoBildaPinpointDriver pinpoint;
    private Transform currentPose = new Transform(0, 0, 0);
    private Transform currentVelocity = new Transform(0, 0, 0);

    private Stopwatch stopwatch;

    public Pinpoint(Transform pose) {
        pinpoint = HardwareManager.claim(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(Constants.pinpointXOffset, Constants.pinpointYOffset);
        pinpoint.recalibrateIMU();
        setPosition(pose);

        stopwatch = new Stopwatch();
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
        if (
                pinpoint.getPosition().getX(DistanceUnit.INCH) == 0
                        && pinpoint.getPosition().getY(DistanceUnit.INCH) == 0
                        && pinpoint.getPosition().getHeading(AngleUnit.RADIANS) == 0
        ) {
            // TODO: Not 100% robust, sometimes I2C errors can happen in the middle of transmission
            // !!! Lynx error !!!
            // Also, if it is legitimately 0, 0, 0 *exactly*, it is probably at the start of the match where
            // a) it probably doesn't matter, this is init
            // and b) the default 0, 0, 0 is probably correct
            return;
        }

        double dt = stopwatch.getDt();

        Transform lastPose = currentPose;
        currentPose = new Transform(pinpoint.getPosition(), pinpoint.getHeading());

        Transform lastVelocity = currentVelocity;
        currentVelocity = new Transform(pinpoint.getVelocity(), pinpoint.getHeadingVelocity());

        // Should be more robust
        if (
                Math.abs((currentPose.getX() - lastPose.getX()) / dt) > 300
                        || Math.abs((currentPose.getY() - lastPose.getY()) / dt) > 300
                        || Math.abs((currentPose.getTheta() - lastPose.getTheta()) / dt) > 6 * Math.PI
                        || Math.abs(currentVelocity.getX()) > 300
                        || Math.abs(currentVelocity.getY()) > 300
                        || Math.abs(currentVelocity.getTheta()) > 6 * Math.PI
        ) {
            // !!! Lynx error !!!
            currentVelocity = lastVelocity;
            currentPose = lastPose;
            stopwatch.undoGetDt();
        }
    }

    public void setPosition(Transform position) {
        pinpoint.setPosition(position != null ? position.toPose2d() : new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        currentPose = position;
    }
}