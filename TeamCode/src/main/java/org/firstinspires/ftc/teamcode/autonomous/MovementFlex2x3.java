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

@Autonomous(name = "Movement Flex 2x3") // Max X : 24 Max Y : 48
public class MovementFlex2x3 extends LinearOpMode {
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
                .easePolynomialTo(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, 0.0);
                    put(DOFs.DOF.Y, 48.0);
                    put(DOFs.DOF.THETA, 360.0);
                }}, 2, 4)
                .easePolynomialTo(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, 24.0);
                    put(DOFs.DOF.Y, 48.0);
                    put(DOFs.DOF.THETA, -360.0);
                }}, 2.5, 6)
                .easeCompoundPolynomialTo(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, 0.0);
                    put(DOFs.DOF.Y, 0.0);
                    put(DOFs.DOF.THETA, 0.0);
                }}, 3, 2, 1, 4)
                .easePolynomialTo(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, 20.33);
                    put(DOFs.DOF.Y, 16.7);
                    put(DOFs.DOF.THETA, 0.0);
                }}, 2, 6)
                .easePolynomialBezierTo(new HashMap<DOFs.DOF, double[]>() {{
                    put(DOFs.DOF.X, new double[] {46.13, 2, 36.65, 27.26, 13, });
                    put(DOFs.DOF.Y, new double[] {14.43, 14, 9.83, 8.26, 7, });
                    put(DOFs.DOF.THETA, new double[] {0.0, 0.0, 0.0, 0.0, 0.0, });
                }}, 4, 6)
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
