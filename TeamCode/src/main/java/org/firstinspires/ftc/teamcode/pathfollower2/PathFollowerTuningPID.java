package org.firstinspires.ftc.teamcode.pathfollower2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.Sensors;

import java.util.HashMap;
import java.util.HashSet;

@TeleOp(name = "Path follower tuning V")
public class PathFollowerTuningPID extends LinearOpMode {
    final double kGoal = 1;
    final double kGuess = 0.1;
    final double kTime = 1;
    final double kAlpha = 0.1;

    final double reinitTime = 2;

    double alpha = 1;

    HashMap<DOFs.DOF, Double> lastPosition = new HashMap<DOFs.DOF, Double>() {
        {
            put(DOFs.DOF.X, 0.0);
            put(DOFs.DOF.Y, 0.0);
            put(DOFs.DOF.THETA, 0.0);
        }
    };

    final HashMap<DOFs.DOF, PID.K> initialGuess = TuningConstants.pid;

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

        double time = 5;
        HashMap<DOFs.DOF, Double> goal = new HashMap<DOFs.DOF, Double>() {
            {
                put(DOFs.DOF.X, 6.0);
                put(DOFs.DOF.Y, 6.0);
                put(DOFs.DOF.THETA, 30.0);
            }
        };

        HashMap<DOFs.DOF, PID.K> guess = new HashMap<>(initialGuess);

        while (opModeIsActive()) {
            HashSet<DOFs.DOF> usedDofs = new HashSet<>();

            double lastRuntime = getRuntime() - 0.1;
            while (!gamepad1.circle) {
                double currentRuntime = getRuntime();
                double dt = currentRuntime - lastRuntime;
                lastRuntime = currentRuntime;

                telemetry.addData("Guess", guess);
                telemetry.addData("Goal", goal);
                telemetry.addData("DOFs", usedDofs);
                telemetry.addData("Time", time);
                telemetry.addData("Alpha", alpha);
                telemetry.addData("Position", lastPosition);
                telemetry.addLine("To modify DOFs: up (X), left (Y), left trigger (THETA)");
                telemetry.addLine(
                        "To modify goal: X & Y (gamepad 2, left stick), to modify THETA (gamepad 2, right stick X)");
                telemetry.addLine("To modify time: gamepad 1 right stick Y.");
                telemetry.addLine("To modify alpha: gamepad 2, right stick Y");
                telemetry.addLine("To modify degree: gamepad 2, up and down");
                telemetry.addLine("Press circle to continue.");
                telemetry.update();

                goal.put(DOFs.DOF.X, goal.get(DOFs.DOF.X) + gamepad2.left_stick_y * dt * kGoal);
                goal.put(DOFs.DOF.Y, goal.get(DOFs.DOF.Y) + gamepad2.left_stick_x * dt * kGoal);
                goal.put(DOFs.DOF.THETA, goal.get(DOFs.DOF.THETA) + gamepad2.right_stick_x * dt * kGoal);
                time -= gamepad1.right_stick_y * dt * kTime;
                alpha -= gamepad2.right_stick_y * dt * kAlpha;

                if (gamepad1.dpad_up) {
                    usedDofs.add(DOFs.DOF.X);
                }
                if (gamepad1.dpad_left) {
                    usedDofs.add(DOFs.DOF.Y);
                }
                if (gamepad1.left_bumper) {
                    usedDofs.add(DOFs.DOF.THETA);
                }
            }

            if (usedDofs.isEmpty())
                continue;

            sensors.odometry.resetPosAndIMU();
            sleep((long) (1000 * reinitTime));

            DOFs dofs = new DOFs(sensors.odometry, motors);

            Path path = new PathBuilder()
                    .startingPoint(goal)
                    .buildSegment()
                    .build();

            PathFollower pathFollower = new PathFollower(path, dofs,
                    guess,
                    new HashMap<DOFs.DOF, PathFollower.K>() {
                        {
                            for (DOFs.DOF dof : DOFs.DOF.values()) put(dof, new PathFollower.K(0, 0, 1));
                        }
                    }, this::isStopRequested);

            pathFollower.run(telemetry, TuningConstants.weights);

            lastPosition = dofs.getPosition();
        }
    }
}
