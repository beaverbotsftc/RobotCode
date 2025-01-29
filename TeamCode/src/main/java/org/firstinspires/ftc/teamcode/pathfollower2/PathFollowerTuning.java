package org.firstinspires.ftc.teamcode.pathfollower2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.Sensors;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;


@TeleOp(name="Path follower tuning")
public class PathFollowerTuning extends LinearOpMode {
    final int trials = 3;
    final int steps = 3;

    final double time = 5;
    final double kBackP = 0.08;
    final double kBackI = 0.01;
    final double kBackD = 0;

    final HashMap<DOFs.DOF, Double> max = new HashMap<DOFs.DOF, Double>() {{
        put(DOFs.DOF.X, 24.0);
        put(DOFs.DOF.Y, 24.0);
        put(DOFs.DOF.THETA, 120.0);
    }};

    final HashMap<DOFs.DOF, Double> initalGuess = new HashMap<DOFs.DOF, Double>() {{
        put(DOFs.DOF.X, 0.1);
        put(DOFs.DOF.Y, 0.1);
        put(DOFs.DOF.THETA, 0.1);
    }};

    final HashMap<DOFs.DOF, Double> resetTolerance = new HashMap<DOFs.DOF, Double>() {{
        put(DOFs.DOF.X, 1.0);
        put(DOFs.DOF.Y, 1.0);
        put(DOFs.DOF.THETA, 15.0);
    }};

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

        HashMap<DOFs.DOF, Double> guess = new HashMap<>(initalGuess);

        Path pathBack = new Path.PathBuilder().isFinished(
                (Double t) -> {
                    for (DOFs.DOF dof : DOFs.DOF.values()) if (Math.abs(new DOFs(sensors.odometry, motors).getPosition().get(dof)) > resetTolerance.get(dof)) return false;
                    return true;
                }
        ).buildSegment().build();


        for (HashSet<DOFs.DOF> dofs : MathUtils.powerSet(new HashSet<>(Arrays.asList(DOFs.DOF.values())))) {
            if (dofs.isEmpty()) continue;
            for (int trial = 0; trial < trials; trial++) {
                for (int step = 1; step <= steps; step++) {
                    final int finalStep = step;
                    Path path = new Path.PathBuilder()
                            .linearTo(new HashMap<DOFs.DOF, Double>() {{
                                for (DOFs.DOF dof : dofs) put(dof, max.get(dof) * (double) finalStep / steps);
                                for (DOFs.DOF dof : MathUtils.compliment(dofs, new HashSet<>(Arrays.asList(DOFs.DOF.values())))) put(dof, 0.0);
                            }}, time)
                            .buildSegment()
                            .build();

                    PathFollower pathFollower = new PathFollower(path, new DOFs(sensors.odometry, motors), new HashMap<DOFs.DOF, PID.K>() {{
                        put(DOFs.DOF.X, new PID.K(1, 0, 0));
                        put(DOFs.DOF.Y, new PID.K(1, 0, 0));
                        put(DOFs.DOF.THETA, new PID.K(1, 0, 0));
                    }}, new HashMap<DOFs.DOF, PathFollower.K>() {{
                        for (DOFs.DOF dof : DOFs.DOF.values()) put(dof, new PathFollower.K(guess.get(dof), 0));
                    }});

                    pathFollower.run(telemetry);

                    for (DOFs.DOF dof : DOFs.DOF.values()) if (dofs.contains(dof)) guess.put(dof, guess.get(dof) * max.get(dof) / new DOFs(sensors.odometry, motors).getPosition().get(dof));

                    pathFollower = new PathFollower(pathBack, new DOFs(sensors.odometry, motors), new HashMap<DOFs.DOF, PID.K>() {{
                        for (DOFs.DOF dof : DOFs.DOF.values()) put(dof, new PID.K(kBackP, kBackI, kBackD));
                    }}, new HashMap<DOFs.DOF, PathFollower.K>() {{
                        for (DOFs.DOF dof : DOFs.DOF.values()) put(dof, new PathFollower.K(0, 1));
                    }});

                    pathFollower.run(telemetry);

                    telemetry.addData("Guess", guess);
                    telemetry.addLine("Press circle to continue.");
                    telemetry.update();

                    while (!gamepad1.circle) {}
                }
            }
        }

        while (opModeIsActive()) {}
    }
}
