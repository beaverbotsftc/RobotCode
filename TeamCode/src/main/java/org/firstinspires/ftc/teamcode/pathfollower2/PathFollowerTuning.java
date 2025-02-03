package org.firstinspires.ftc.teamcode.pathfollower2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.Sensors;

@TeleOp(name = "Path follower tuning")
public class PathFollowerTuning extends LinearOpMode {
    final double kGoal = 5;
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

    final HashMap<DOFs.DOF, Double> initalGuess = new HashMap<DOFs.DOF, Double>() {
        {
            put(DOFs.DOF.X, 0.0222);
            put(DOFs.DOF.Y, 0.1);
            put(DOFs.DOF.THETA, 0.0191);
        }
    };

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
                put(DOFs.DOF.X, 24.0);
                put(DOFs.DOF.Y, 24.0);
                put(DOFs.DOF.THETA, 360.0);
            }
        };

        HashMap<DOFs.DOF, Double> guess = new HashMap<>(initalGuess);

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
                        "To modify guess: X & Y (gamepad 1, left stick), to modify THETA (gamepad 1, right stick X)");
                telemetry.addLine(
                        "To modify goal: X & Y (gamepad 2, left stick), to modify THETA (gamepad 2, right stick X)");
                telemetry.addLine("To modify time: gamepad 1 right stick Y.");
                telemetry.addLine("To modify alpha: gamepad 2, right stick Y");
                telemetry.addLine("Press circle to continue.");
                telemetry.update();

                guess.put(DOFs.DOF.X,
                        guess.get(DOFs.DOF.X)
                                + gamepad1.left_stick_y * dt
                                * kGuess); // X and Y reversed because X is forward on the bot for some reason
                guess.put(DOFs.DOF.Y, guess.get(DOFs.DOF.Y) + gamepad1.left_stick_x * dt * kGuess);
                guess.put(DOFs.DOF.THETA, guess.get(DOFs.DOF.THETA) + gamepad1.right_stick_x * dt * kGuess);
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
                    .linearTo(
                            new HashMap<DOFs.DOF, Double>() {
                                {
                                    for (DOFs.DOF dof : usedDofs) put(dof, goal.get(dof));
                                    for (DOFs.DOF dof : MathUtils.compliment(
                                            usedDofs, new HashSet<>(Arrays.asList(DOFs.DOF.values()))))
                                        put(dof, 0.0);
                                }
                            },
                            time)
                    .buildSegment()
                    .build();

            PathFollower pathFollower = new PathFollower(path, dofs,
                    new HashMap<DOFs.DOF, PID.K>() {
                        {
                            put(DOFs.DOF.X, new PID.K(1, 0, 0));
                            put(DOFs.DOF.Y, new PID.K(1, 0, 0));
                            put(DOFs.DOF.THETA, new PID.K(1, 0, 0));
                        }
                    },
                    new HashMap<DOFs.DOF, PathFollower.K>() {
                        {
                            for (DOFs.DOF dof : DOFs.DOF.values()) put(dof, new PathFollower.K(guess.get(dof), 0));
                        }
                    }, this::isStopRequested);

            pathFollower.run(telemetry);

            for (DOFs.DOF dof : DOFs.DOF.values())
                if (usedDofs.contains(dof))
                    guess.put(dof,
                            alpha * guess.get(dof) * goal.get(dof) / dofs.getPosition().get(dof)
                                    + (1 - alpha) * guess.get(dof));

            lastPosition = dofs.getPosition();
        }
    }
}
