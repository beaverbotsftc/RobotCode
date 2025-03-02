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

@Autonomous(name = "SpecimenAuto5")
public class SampleSpecimenAuto5 extends LinearOpMode {
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
                        () ->
                                sensors.odometry.setPosition(new Pose2D(DistanceUnit.INCH, 65.480000, 17.300000, AngleUnit.DEGREES, -179.000000))
                )
                .startingPoint(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.Y, 17.300000);
                    put(DOFs.DOF.X, 65.480000);
                    put(DOFs.DOF.THETA, -179.000000);
                }})
                .easeCompoundPolynomialTo(
                        new HashMap<DOFs.DOF, Double>() {{
                            put(DOFs.DOF.Y, 6.150000);
                            put(DOFs.DOF.X, 33.210000);
                            put(DOFs.DOF.THETA, -179.000000);
                        }},
                        2.000000, 2.000000, 1.000000, 3.000000
                )
                .addTime(1.000000)
                .easeCompoundPolynomialBezierTo(
                        new HashMap<DOFs.DOF, double[]>() {{
                            put(DOFs.DOF.Y, new double[] {23.940000, 34.400000, -36.570000});
                            put(DOFs.DOF.X, new double[] {42.410000, 37.170000, -23.080000});
                            put(DOFs.DOF.THETA, new double[] {90.000000, 45.000000, 0.000000});
                        }},
                        3.000000, 1.000000, 1.000000, 1.000000
                )
                .easeCompoundPolynomialBezierTo(
                        new HashMap<DOFs.DOF, double[]>() {{
                            put(DOFs.DOF.Y, new double[] {38.740000, 55.179000, 46.890000});
                            put(DOFs.DOF.X, new double[] {8.990000, 12.950000, 51.100000});
                            put(DOFs.DOF.THETA, new double[] {0.000000, 0.000000, 0.000000});
                        }},
                        1.000000, 2.000000, 1.000000, 1.000000
                )
                .easeCompoundPolynomialBezierTo(
                        new HashMap<DOFs.DOF, double[]>() {{
                            put(DOFs.DOF.Y, new double[] {49.880000, 62.780000, 56.940000});
                            put(DOFs.DOF.X, new double[] {-19.510000, 2.179000, 52.530000});
                            put(DOFs.DOF.THETA, new double[] {0.000000, 0.000000, 0.000000});
                        }},
                        2.000000, 2.000000, 1.000000, 2.000000
                )
                .easeCompoundPolynomialBezierTo(
                        new HashMap<DOFs.DOF, double[]>() {{
                            put(DOFs.DOF.Y, new double[] {57.770000, 67.260000, 63.330000});
                            put(DOFs.DOF.X, new double[] {-7.630000, 2.660000, 50.940000});
                            put(DOFs.DOF.THETA, new double[] {0.000000, 0.000000, 0.000000});
                        }},
                        2.000000, 2.000000, 1.000000, 2.000000
                )
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
