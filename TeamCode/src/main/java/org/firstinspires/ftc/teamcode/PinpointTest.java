package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class PinpointTest extends LinearOpMode {
    public void runOpMode() {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(-101.6, -101.6, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();
            telemetry.addData("X", pinpoint.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y", pinpoint.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading", pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
            telemetry.update();
        }
    }
}
