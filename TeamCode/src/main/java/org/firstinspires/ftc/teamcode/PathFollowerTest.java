package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pathfollower.PIDCoefficients;
import org.firstinspires.ftc.teamcode.pathfollower.Path;
import org.firstinspires.ftc.teamcode.pathfollower.PathFollower;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.Sensors;

import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Path follower test")
public class PathFollowerTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        Motors motors = new Motors();
        Sensors sensors = new Sensors();
        motors.init(hardwareMap);
        sensors.init(hardwareMap);

        Path path = new Path((Double t)->0.0, (Double t)->0.0, (Double t)->0.0);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        PathFollower pathFollower = new PathFollower(
                telemetry,
                runtime.now(TimeUnit.NANOSECONDS),
                path,
                motors,
                sensors.odometry,
                new PathFollower.RawAndPIDGains(1, 1),
                new PathFollower.RawAndPIDGains(1, 1),
                new PathFollower.RawAndPIDGains(1, 1),
                new PIDCoefficients(1, 0, 0.5),
                new PIDCoefficients(1, 0, 0.5),
                new PIDCoefficients(0.3, 0, 0.1),
                new PathFollower.AxisGains(1, 1, 1),
                new PathFollower.MotorGains(1, 1, 1, 1)
                );

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            sensors.odometry.bulkUpdate();
            telemetry.addData("time", runtime.now(TimeUnit.MILLISECONDS));
            pathFollower.run(runtime.now(TimeUnit.NANOSECONDS));
            telemetry.addData("X Pos", sensors.odometry.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y Pos", sensors.odometry.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Theta Pos", sensors.odometry.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
