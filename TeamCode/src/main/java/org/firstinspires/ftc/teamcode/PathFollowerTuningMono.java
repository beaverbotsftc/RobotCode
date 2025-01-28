package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.Sensors;
import org.firstinspires.ftc.teamcode.pathfollower2.DOFs;
import org.firstinspires.ftc.teamcode.pathfollower2.PID;
import org.firstinspires.ftc.teamcode.pathfollower2.Path;
import org.firstinspires.ftc.teamcode.pathfollower2.PathFollower;

import java.util.HashMap;


@TeleOp(name="Path follower tuning mono")
public class PathFollowerTuningMono extends LinearOpMode {
    @Override
    public void runOpMode() {

        Motors motors = new Motors();
        Sensors sensors = new Sensors();
        motors.init(hardwareMap);
        sensors.init(hardwareMap);

        Path pathTo = new Path.PathBuilder()
                .linearTo(new HashMap<DOFs.DOF, Double>() {{
                    put(DOFs.DOF.X, 48.0);
                    put(DOFs.DOF.Y, 0.0);
                    put(DOFs.DOF.THETA, 0.0);
                }}, 5)
                .buildSegment()
                .build();

        Path pathBack = new Path.PathBuilder()
                .isFinished((Double t) -> Math.abs(sensors.odometry.getPosition().getX(DistanceUnit.INCH)) < 1 && Math.abs(sensors.odometry.getPosition().getY(DistanceUnit.INCH)) < 1 && Math.abs(sensors.odometry.getPosition().getHeading(AngleUnit.DEGREES)) < 5 )
                .buildSegment()
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.speak("I have been initialized!");
        telemetry.update();

        waitForStart();

        double guessX = 0.0176;
        double guessY = 0.03;
        double guessTheta = 0.03;

        final double finalGuessX = guessX;
        final double finalGuessY = guessY;
        final double finalGuessTheta = guessTheta;
        PathFollower pathFollower = new PathFollower(pathTo, new DOFs(sensors.odometry, motors), new HashMap<DOFs.DOF, PID.K>() {{
            put(DOFs.DOF.X, new PID.K(1, 0, 0));
            put(DOFs.DOF.Y, new PID.K(1, 0, 0));
            put(DOFs.DOF.THETA, new PID.K(1, 0, 0));
        }}, new HashMap<DOFs.DOF, PathFollower.K>() {{
            put(DOFs.DOF.X, new PathFollower.K(finalGuessX, 0));
            put(DOFs.DOF.Y, new PathFollower.K(finalGuessY, 0));
            put(DOFs.DOF.THETA, new PathFollower.K(finalGuessTheta, 0));
        }});

        pathFollower.run(telemetry);

        guessX *= 48.0 / sensors.odometry.getPosition().getX(DistanceUnit.INCH);

        pathFollower = new PathFollower(pathBack, new DOFs(sensors.odometry, motors), new HashMap<DOFs.DOF, PID.K>() {{
            put(DOFs.DOF.X, new PID.K(1, 0.5, 0));
            put(DOFs.DOF.Y, new PID.K(1, 0.5, 0));
            put(DOFs.DOF.THETA, new PID.K(1, 0.5, 0));
        }}, new HashMap<DOFs.DOF, PathFollower.K>() {{
            put(DOFs.DOF.X, new PathFollower.K(0, 0.01));
            put(DOFs.DOF.Y, new PathFollower.K(0, 0.01));
            put(DOFs.DOF.THETA, new PathFollower.K(0, 0.01));
        }});

        pathFollower.run(telemetry);

        telemetry.addData("Guess X", guessX);
        telemetry.addData("Guess Y", guessY);
        telemetry.addData("Guess Theta", guessTheta);
        telemetry.update();

        while (opModeIsActive()) {}
    }
}
