package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Sensor test")
public class SensorTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Sensors sensors = new Sensors();

    @Override
    public void runOpMode() {
        sensors.init(telemetry, hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Time elapsed", runtime.now(TimeUnit.SECONDS));
            telemetry.addLine("Pinpoint");
            sensors.odometry.bulkUpdate();
            telemetry.addData("X position (inches)", sensors.odometry.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y position (inches)", sensors.odometry.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading (degrees)", sensors.odometry.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("X velocity (inches/second)", sensors.odometry.getVelocity().getX(DistanceUnit.INCH));
            telemetry.addData("Y velocity (inches/second)", sensors.odometry.getVelocity().getY(DistanceUnit.INCH));
            telemetry.addData("Heading velocity (degrees/second)", sensors.odometry.getVelocity().getHeading(AngleUnit.DEGREES));
            telemetry.addLine();
            telemetry.update();
        }
    }
}