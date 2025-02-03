package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.Sensors;
import org.firstinspires.ftc.teamcode.pathfollower2.DOFs;
import org.firstinspires.ftc.teamcode.pathfollower2.MathUtils;
import org.firstinspires.ftc.teamcode.pathfollower2.PID;
import org.firstinspires.ftc.teamcode.pathfollower2.Path;
import org.firstinspires.ftc.teamcode.pathfollower2.PathBuilder;
import org.firstinspires.ftc.teamcode.pathfollower2.PathFollower;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;

@Autonomous(name = "Specimen Park")
public class ParkSpecimen extends LinearOpMode {
    @Override
    public void runOpMode() {
        Motors motors = new Motors();
        Sensors sensors = new Sensors();
        motors.init(hardwareMap);
        sensors.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.speak("I have been initialized!");
        telemetry.update();
        waitForStart();

        Path path = new PathBuilder()
                .easeCompoundPolynomialTo(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, 0.0);
                    put(DOFs.DOF.Y, -48.0);
                    put(DOFs.DOF.THETA, 0.0);
                }}, 2, 1.2, 1, 2)
                .buildSegment()
                .build();

        PathFollower pathFollower = new PathFollower(path, new DOFs(sensors.odometry, motors), new HashMap<DOFs.DOF, PID.K>() {{
            put(DOFs.DOF.X, new PID.K(0.1, 0, 0));
            put(DOFs.DOF.Y, new PID.K(0.1, 0, 0));
            put(DOFs.DOF.THETA, new PID.K(0.1, 0, 0));
        }}, new HashMap<DOFs.DOF, PathFollower.K>() {{
            put(DOFs.DOF.X, new PathFollower.K(TuningConstants.x, 1));
            put(DOFs.DOF.Y, new PathFollower.K(TuningConstants.y, 1));
            put(DOFs.DOF.THETA, new PathFollower.K(TuningConstants.theta, 1));
        }}, this::isStopRequested);

        pathFollower.run(telemetry);
    }
}
