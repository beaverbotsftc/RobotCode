package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Sensor test")
public class SensorTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Sensors sensors = new Sensors();

    @Override
    public void runOpMode() {
        sensors.init();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addLine("Pinpoint");
            telemetry.addData("Time elapsed", runtime.now(TimeUnit.SECONDS));
            telemetry.addData("X position", sensors.odometry.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y position", sensors.odometry.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading", sensors.odometry.getHeading());
            telemetry.addData("X velocity", sensors.odometry.getVelocity().getX(DistanceUnit.INCH));
            telemetry.addData("Y velocity", sensors.odometry.getVelocity().getY(DistanceUnit.INCH));
            telemetry.addData("Heading velocity", sensors.odometry.getHeadingVelocity());
            telemetry.addLine();
            telemetry.update();
        }
    }
}
