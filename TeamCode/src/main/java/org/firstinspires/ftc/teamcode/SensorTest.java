package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.collections.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Sensor test")
public class SensorTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Sensors sensors = new Sensors();

    @Override
    public void runOpMode() {
        sensors.init(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Time elapsed", runtime.now(TimeUnit.SECONDS));

            // Pinpoint
            telemetry.addLine("Pinpoint");
            sensors.odometry.bulkUpdate();
            telemetry.addData("X position (inches)", sensors.odometry.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y position (inches)", sensors.odometry.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading (degrees)", sensors.odometry.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("X velocity (inches/second)", sensors.odometry.getVelocity().getX(DistanceUnit.INCH));
            telemetry.addData("Y velocity (inches/second)", sensors.odometry.getVelocity().getY(DistanceUnit.INCH));
            telemetry.addData("Heading velocity (degrees/second)", sensors.odometry.getVelocity().getHeading(AngleUnit.DEGREES));

            // Color sensor
            telemetry.addLine("Color sensor");
            telemetry.addData("Red", (float) sensors.color.red() / 255);
            telemetry.addData("Green", (float) sensors.color.green() / 255);
            telemetry.addData("Blue", (float) sensors.color.blue() / 255);
            telemetry.addLine(Math.sqrt(
                    Math.pow(sensors.color.red(), 2) + Math.pow(sensors.color.green(), 2) + Math.pow(sensors.color.blue(), 2))
                    >= 220 ? "Bright" : "Dark");

            telemetry.update();
        }
    }
}