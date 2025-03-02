package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.Sensors;
import org.firstinspires.ftc.teamcode.pathfollower2.DOFs;
import org.firstinspires.ftc.teamcode.pathfollower2.PID;
import org.firstinspires.ftc.teamcode.pathfollower2.Path;
import org.firstinspires.ftc.teamcode.pathfollower2.PathBuilder;
import org.firstinspires.ftc.teamcode.pathfollower2.PathFollower;
import org.firstinspires.ftc.teamcode.pathfollower2.TuningConstants;

import java.util.HashMap;

@Autonomous(name = "Showcase") // Max X : 24 Max Y : 48
public class Showcase extends LinearOpMode {
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
                /*
                .easePolynomialTo(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, 35.8);
                    put(DOFs.DOF.Y, 17.7);
                    put(DOFs.DOF.THETA, 0.0);
                }}, 2, 4)
                .easePolynomialBezierTo(new HashMap<DOFs.DOF, double[]>() {{
                    put(DOFs.DOF.X, new double[] {48, 2.6, 36.65, 27.26, 13, });
                    put(DOFs.DOF.Y, new double[] {32.63, 21.9, 9.83, 8.26, 7, });
                    put(DOFs.DOF.THETA, new double[] {0.0, 0.0, 0.0, 0.0, 0.0, });
                }}, 1.9, 5)
                */
                .easePolynomialTo(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, 6.18);
                    put(DOFs.DOF.Y, 44.1);
                    put(DOFs.DOF.THETA, 0.0);
                }}, 2, 8)
                .easePolynomialBezierTo(new HashMap<DOFs.DOF, double[]>() {{
                    put(DOFs.DOF.X, new double[] {45, 3.8, 26, 37, 26, });
                    put(DOFs.DOF.Y, new double[] {14, 3.75, 36, 4, 2, });
                    put(DOFs.DOF.THETA, new double[] {0.0, 0.0, 0.0, 0.0, 0.0, });
                }}, 2, 8)
               .buildSegment()
                .build();

        PathFollower pathFollower = new PathFollower(path, new DOFs(sensors.odometry, motors), new HashMap<DOFs.DOF, PID.K>() {{
            put(DOFs.DOF.X, new PID.K(0.2, 0.1, 0.02));
            put(DOFs.DOF.Y, new PID.K(0.2, 0.1, 0.02));
            put(DOFs.DOF.THETA, new PID.K(0.25, 0.05, 0.0025));
        }}, new HashMap<DOFs.DOF, PathFollower.K>() {{
            for (DOFs.DOF dof : DOFs.DOF.values()) put(dof, new PathFollower.K(TuningConstants.v.get(dof), 0, 1));
        }}, this::isStopRequested);

        pathFollower.run(telemetry);
    }
}
