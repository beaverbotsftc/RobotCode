package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.Sensors;
import org.firstinspires.ftc.teamcode.pathfollower.PIDCoefficients;
import org.firstinspires.ftc.teamcode.pathfollower.PathBuilder;
import org.firstinspires.ftc.teamcode.pathfollower2.DOFs;
import org.firstinspires.ftc.teamcode.pathfollower2.PID;
import org.firstinspires.ftc.teamcode.pathfollower2.Path;
import org.firstinspires.ftc.teamcode.pathfollower2.PathComponent;
import org.firstinspires.ftc.teamcode.pathfollower2.PathFollower;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.function.Function;


@TeleOp(name="Path follower test 2")
public class PathFollowerTest2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Motors motors = new Motors();
        Sensors sensors = new Sensors();
        motors.init(hardwareMap);
        sensors.init(hardwareMap);

        new Thread(
            new PathFollower(
                new Path(
                        new ArrayList<>(Arrays.asList(new PathComponent[]{
                                new PathComponent((Double t) -> t > 5, new HashMap<DOFs.DOF, Function<Double, Double>>() {{
                                    put(DOFs.DOF.X, (Double t) -> 0.0);
                                    put(DOFs.DOF.Y, (Double t) -> 0.0);
                                    put(DOFs.DOF.THETA, (Double t) -> 0.0);
                                }}),
                                new PathComponent((Double t) -> t > 5, new HashMap<DOFs.DOF, Function<Double, Double>>() {{
                                    put(DOFs.DOF.X, (Double t) -> 0.0);
                                    put(DOFs.DOF.Y, (Double t) -> 0.0);
                                    put(DOFs.DOF.THETA, (Double t) -> 0.0);
                                }})
                        }))
                ),
                new DOFs(motors, sensors.odometry),
                new HashMap<DOFs.DOF, PID.PIDCoefficients>() {{
                    put(DOFs.DOF.X, new PID.PIDCoefficients(1, 0, 0));
                    put(DOFs.DOF.Y, new PID.PIDCoefficients(1, 0, 0));
                    put(DOFs.DOF.THETA, new PID.PIDCoefficients(1, 0, 0));
                }},
                new HashMap<DOFs.DOF, PathFollower.PathFollowerCoefficients>() {{
                    put(DOFs.DOF.X, new PathFollower.PathFollowerCoefficients(1, 1));
                    put(DOFs.DOF.Y, new PathFollower.PathFollowerCoefficients(1, 1));
                    put(DOFs.DOF.THETA, new PathFollower.PathFollowerCoefficients(1, 1));
                }}
            )
        ).start();

        waitForStart();

        while (opModeIsActive()) {
            sensors.odometry.update();
            telemetry.addData("X Pos", sensors.odometry.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y Pos", sensors.odometry.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Theta Pos", sensors.odometry.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
