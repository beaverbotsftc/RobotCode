package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.Sensors;
import org.firstinspires.ftc.teamcode.pathfollower2.DOFs;
import org.firstinspires.ftc.teamcode.pathfollower2.PID;
import org.firstinspires.ftc.teamcode.pathfollower2.Path;
import org.firstinspires.ftc.teamcode.pathfollower2.PathBuilder;
import org.firstinspires.ftc.teamcode.pathfollower2.PathFollower;
import org.firstinspires.ftc.teamcode.pathfollower2.TuningConstants;

import java.util.HashMap;

@Autonomous(name = "Testing")
public class Testing extends LinearOpMode {
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
                .onInitBlocking(
                        () -> sensors.odometry.setPosition(new Pose2D(DistanceUnit.INCH, -65.530000, -6.700000, AngleUnit.DEGREES, -180.000000))
                )
                .startingPoint(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, -65.530000);
                    put(DOFs.DOF.Y, -6.700000);
                    put(DOFs.DOF.THETA, -180.000000);
                }})
                .easeCompoundPolynomialTo(
                        new HashMap<DOFs.DOF, Double>() {{
                            put(DOFs.DOF.X, -33.000000);
                            put(DOFs.DOF.Y, -6.700000);
                            put(DOFs.DOF.THETA, -180.000000);
                        }},
                        2.000000, 2.000000, 1.000000, 4.000000
                )
                .buildSegment()
                .onInitBlocking(
                        () -> sensors.odometry.setPosition(new Pose2D(DistanceUnit.INCH, -33.000000, -6.700000, AngleUnit.DEGREES, -180.000000))
                )
                .startingPoint(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, -33.000000);
                    put(DOFs.DOF.Y, -6.700000);
                    put(DOFs.DOF.THETA, -180.000000);
                }})
                .addTime(1.000000)
                .buildSegment()
                .startingPoint(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, -33.000000);
                    put(DOFs.DOF.Y, -6.700000);
                    put(DOFs.DOF.THETA, -180.000000);
                }})
                .easeCompoundPolynomialBezierTo(
                        new HashMap<DOFs.DOF, double[]>() {{
                            put(DOFs.DOF.X, new double[] {-54.640000, -42.250000, -11.960000, -13.980000});
                            put(DOFs.DOF.Y, new double[] {-19.360000, -45.440000, -27.470000, -46.730000});
                            put(DOFs.DOF.THETA, new double[] {-120.000000, -30.000000, 0.000000, 0.000000});
                        }},
                        2.000000, 2.000000, 1.000000, 4.500000
                )
                .easeCompoundPolynomialTo(
                        new HashMap<DOFs.DOF, Double>() {{
                            put(DOFs.DOF.X, -49.390000);
                            put(DOFs.DOF.Y, -47.320000);
                            put(DOFs.DOF.THETA, 0.000000);
                        }},
                        2.000000, 2.000000, 1.000000, 3.000000
                )
                .buildSegment()
                .build();

        PathFollower pathFollower = new PathFollower(path, new DOFs(sensors.odometry, motors), new HashMap<DOFs.DOF, PID.K>() {{
            put(DOFs.DOF.X, new PID.K(0.2, 0.2, 0.03));
            put(DOFs.DOF.Y, new PID.K(0.2, 0.2, 0.03));
            put(DOFs.DOF.THETA, new PID.K(0.25, 0.05, 0.0025));
        }}, new HashMap<DOFs.DOF, PathFollower.K>() {{
            for (DOFs.DOF dof : DOFs.DOF.values()) put(dof, new PathFollower.K(TuningConstants.v.get(dof), 0, 1));
        }}, this::isStopRequested);


        pathFollower.run(telemetry, TuningConstants.weights);
    }
}
