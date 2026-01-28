
package org.firstinspires.ftc.teamcode.subsystems.localizer;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

import java.util.List;

public final class Pinpoint implements Localizer, Subsystem {
    private static final double IN_TO_MM = 25.4;

    private GoBildaPinpointDriver pinpoint;
    private DrivetrainState currentPose = new DrivetrainState(0, 0, 0);
    private DrivetrainState currentVelocity = new DrivetrainState(0, 0, 0);

    public Pinpoint(DrivetrainState pose) {
        pinpoint = HardwareManager.claim(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(Constants.pinpointXOffset * IN_TO_MM, Constants.pinpointYOffset * IN_TO_MM);
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
        if (
                pinpoint.getPosition().getX(DistanceUnit.INCH) == 0
                        && pinpoint.getPosition().getY(DistanceUnit.INCH) == 0
                        && pinpoint.getPosition().getHeading(AngleUnit.RADIANS) == 0
        ) {
            // !!! PINPOINT UNPLUGGED OR SOMETHING; HOPE IT IS TEMPORARY, JUST USE PRIOR DATA IN THE MEANTIME !!!
            // Also, if it is legitimately 0, 0, 0 *exactly*, it is probably at the start of the match where
            // a) it probably doesn't matter, this is init
            // and b) the default 0, 0, 0 is probably correct
            return;
        }
        currentPose = new DrivetrainState(pinpoint.getPosition(), pinpoint.getHeading());
        currentVelocity = new DrivetrainState(pinpoint.getVelocity(), pinpoint.getHeadingVelocity());
    }

    public void setPosition(DrivetrainState position) {
        pinpoint.setPosition(position != null ? position.toPose2d() : new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        currentPose = position;
    }

    public double wind(double theta) {
        return Localizer.wind(theta, getPosition().getTheta());
    }
}