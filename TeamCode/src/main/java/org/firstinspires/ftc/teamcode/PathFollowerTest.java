package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        Path path = new Path((Double t) -> Math.sin(t), (Double t) -> t);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        PathFollower pathFollower = new PathFollower(runtime.now(TimeUnit.NANOSECONDS), path, motors, sensors.odometry);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            pathFollower.apply(runtime.now(TimeUnit.NANOSECONDS));
        }
    }
}
