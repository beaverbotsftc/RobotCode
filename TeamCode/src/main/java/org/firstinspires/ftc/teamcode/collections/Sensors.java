package org.firstinspires.ftc.teamcode.collections;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

public class Sensors {
    public GoBildaPinpointDriver odometry;

    public void init(HardwareMap hardwareMap) {
        // Pinpoint
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        /*
            Set the odometry pod positions relative to the point that the odometry computer tracks around.
            The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is. Left of the center is a positive number, right of center is a negative number.
            The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is. forward of center is a positive number, backwards is a negative number.
        */

        // TODO: Test positive and negative, and forward and backward
        //odometry.setOffsets(76.0, 79.81);
        odometry.setOffsets(0.0, 0.0);

        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odometry.resetPosAndIMU();
        odometry.update();
    }
}
