package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.collections.Sensors;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Pinpoint Test")
public class GobildaPinpointTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Sensors sensors = new Sensors();

    @Override
    public void runOpMode() {
        sensors.init(hardwareMap);

        GoBildaPinpointDriver odometry = sensors.odometry;

        waitForStart();
        runtime.reset();

        double initalPositionX = odometry.getPosition().getX(DistanceUnit.INCH);
        double initalPositionY = odometry.getPosition().getY(DistanceUnit.INCH);
        double initalPositionTheta = odometry.getPosition().getHeading(AngleUnit.DEGREES);

        sensors.odometry.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -180));
        while (opModeIsActive()) {
            odometry.update();
            telemetry.addData("Inital Position X", initalPositionX);
            telemetry.addData("Inital Position Y", initalPositionY);
            telemetry.addData("Inital Position Theta", initalPositionTheta);
            telemetry.addLine();
            telemetry.addData("Position X", odometry.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Position Y", odometry.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Position Theta", odometry.getHeading() / Math.PI * 180);
            telemetry.update();
        }
    }
}