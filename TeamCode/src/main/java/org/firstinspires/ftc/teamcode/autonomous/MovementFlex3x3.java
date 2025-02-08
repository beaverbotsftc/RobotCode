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

import java.util.HashMap;

@Autonomous(name = "Movement Flex 3x3") // Max X & Y : 48
public class MovementFlex3x3 extends LinearOpMode {
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
                    put(DOFs.DOF.X, 48.0);
                    put(DOFs.DOF.Y, 48.0);
                    put(DOFs.DOF.THETA, -360.0);
                }}, 2.5, 6)
                .easeCompoundPolynomialTo(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, 0.0);
                    put(DOFs.DOF.Y, 0.0);
                    put(DOFs.DOF.THETA, 0.0);
                }}, 3, 2, 1, 4)
                .easePolynomialTo(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, 25.2);
                    put(DOFs.DOF.Y, 34.2);
                    put(DOFs.DOF.THETA, 0.0);
                }}, 3, 6)
                .easePolynomialBezierTo(new HashMap<DOFs.DOF, double[]>() {{
                    put(DOFs.DOF.X, new double[] {6.93, 6.6, 39, 39.33, 25.3, });
                    put(DOFs.DOF.Y, new double[] {32.4, 4.1, 12, 35.46, 34.25, });
                    put(DOFs.DOF.THETA, new double[] {0.0, 0.0, 0.0, 0.0, 0.0, });
                }}, 3, 6)
                .easePolynomialBezierTo(new HashMap<DOFs.DOF, double[]>() {{
                    put(DOFs.DOF.X, new double[] {13, 7.5, 40.5, 23, 24, });
                    put(DOFs.DOF.Y, new double[] {40.4, 37.3, 17, 7, 24, });
                    put(DOFs.DOF.THETA, new double[] {0.0, 0.0, 0.0, 0.0, 0.0, });
                }}, 2, 4)
                .buildSegment()
                .build();

        PathFollower pathFollower = new PathFollower(path, new DOFs(sensors.odometry, motors), new HashMap<DOFs.DOF, PID.K>() {{
            put(DOFs.DOF.X, new PID.K(0.2, 0.1, 0.02));
            put(DOFs.DOF.Y, new PID.K(0.2, 0.1, 0.02));
            put(DOFs.DOF.THETA, new PID.K(0.25, 0.05, 0.0025));
        }}, new HashMap<DOFs.DOF, PathFollower.K>() {{
            put(DOFs.DOF.X, new PathFollower.K(TuningConstants.x, 1));
            put(DOFs.DOF.Y, new PathFollower.K(TuningConstants.y, 1));
            put(DOFs.DOF.THETA, new PathFollower.K(TuningConstants.theta, 1));
        }}, this::isStopRequested);

        pathFollower.run(telemetry);
    }
}
