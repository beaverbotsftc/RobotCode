package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.SequentialCommandGroup;
import org.beaverbots.beavertracking.HolonomicFollowPath;
import org.beaverbots.beavertracking.Localizer;
import org.beaverbots.beavertracking.Locomotion;
import org.beaverbots.beavertracking.PIDF;
import org.beaverbots.beavertracking.PIDFAxis;
import org.beaverbots.beavertracking.Path;
import org.beaverbots.beavertracking.PathAxis;
import org.beaverbots.beavertracking.RamseteFollowPath;
import org.beaverbots.beavertracking.RamsetePathTracker;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

@TeleOp
public class TrackingTest extends CommandRuntimeOpMode {
    public static class SimulatedRobot implements Localizer, Locomotion {
        private static final double MAX_LINEAR_ACCEL = 5.0; // units/s^2
        private static final double MAX_ANGULAR_ACCEL = Math.PI; // rad/s^2
        private static final double SENSOR_NOISE_STD_DEV = 0.01; // units

        private final List<Double> position = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0));
        private final List<Double> velocity = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0));
        private List<Double> targetVelocity = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0));

        private final Random noiseGenerator = new Random();

        public void update(double dt) {
            // Simulate Inertia: Gradually move current velocity towards target velocity
            // For vx
            double current_vx = velocity.get(0);
            double target_vx = targetVelocity.get(0);
            double max_dvx = MAX_LINEAR_ACCEL * dt;
            velocity.set(0, current_vx + Math.max(-max_dvx, Math.min(max_dvx, target_vx - current_vx)));

            // For vy
            double current_vy = velocity.get(1);
            double target_vy = targetVelocity.get(1);
            double max_dvy = MAX_LINEAR_ACCEL * dt;
            velocity.set(1, current_vy + Math.max(-max_dvy, Math.min(max_dvy, target_vy - current_vy)));

            // For vtheta
            double current_vtheta = velocity.get(2);
            double target_vtheta = targetVelocity.get(2);
            double max_dvtheta = MAX_ANGULAR_ACCEL * dt;
            velocity.set(2, current_vtheta + Math.max(-max_dvtheta, Math.min(max_dvtheta, target_vtheta - current_vtheta)));

            position.set(0, position.get(0) + velocity.get(0) * dt);
            position.set(1, position.get(1) + velocity.get(1) * dt);
            position.set(2, position.get(2) + velocity.get(2) * dt);
        }

        @Override
        public List<Double> getPosition() {
            return Arrays.asList(
                    position.get(0) + noiseGenerator.nextGaussian() * SENSOR_NOISE_STD_DEV,
                    position.get(1) + noiseGenerator.nextGaussian() * SENSOR_NOISE_STD_DEV,
                    position.get(2) + noiseGenerator.nextGaussian() * SENSOR_NOISE_STD_DEV
            );
        }

        public List<Double> getTruePosition() {
            return new ArrayList<>(position);
        }

        @Override
        public void move(List<Double> movement) {
            if (movement.size() == 3) {
                // Holonomic output: [vx, vy, v_theta]
                // The target velocity is directly given.
                this.targetVelocity = movement;
            } else if (movement.size() == 2) {
                // Ramsete output: [v, omega] (linear velocity, angular velocity)
                double v = movement.get(0);
                double omega = movement.get(1);
                double currentTheta = position.get(2); // Use true theta for conversion

                double target_vx = v * Math.cos(currentTheta);
                double target_vy = v * Math.sin(currentTheta);

                this.targetVelocity = Arrays.asList(target_vx, target_vy, omega);
            }
        }
    }

    private SimulatedRobot simulatedRobot;
    private Command testSequence;
    private ElapsedTime loopTimer = new ElapsedTime();
    private int logCounter = 0;

    @Override
    protected void onInit() {
        RobotLog.d("onInit called");

        simulatedRobot = new SimulatedRobot();
        register(simulatedRobot);

        Path holonomicPath = new Path(
                Arrays.asList(
                        new PathAxis(t -> 10 * Math.sin(t * Math.PI / 4), 0, 8), // x: moves in a sine wave
                        new PathAxis(t -> 5 * t, 0, 8),                          // y: moves linearly
                        new PathAxis(t -> 0, 0, 8)                               // theta: stays at 0
                ),
                t -> t >= 8 // Finish after 8 seconds
        );
        PIDF holonomicPIDF = new PIDF(Arrays.asList(
                new PIDFAxis(new PIDFAxis.K(0.8, 0, 0.1, 1, 1, 5, 0.02)), // x-axis PIDF
                new PIDFAxis(new PIDFAxis.K(0.8, 0, 0.1, 1, 1, 5, 0.02)), // y-axis PIDF
                new PIDFAxis(new PIDFAxis.K(1.0, 0, 0.15, 1, 1, 3, 0.02)) // theta-axis PIDF
        ));
        Command followHolonomic = new HolonomicFollowPath(holonomicPath, holonomicPIDF, simulatedRobot, simulatedRobot);


        double radius = 10.0;
        Path ramsetePath = new Path(
                Arrays.asList(
                        new PathAxis(t -> radius * Math.cos(t), 0, 2 * Math.PI), // x(t)
                        new PathAxis(t -> radius * Math.sin(t), 0, 2 * Math.PI), // y(t)
                        new PathAxis(t -> t + Math.PI / 2, 0, 2 * Math.PI)       // theta(t) is tangent to the circle
                ),
                t -> t >= 2 * Math.PI // Finish after one full circle
        );
        RamsetePathTracker.K ramseteK = new RamsetePathTracker.K(2.0, 0.7);
        Command followRamsete = new RamseteFollowPath(ramsetePath, ramseteK, simulatedRobot, simulatedRobot);


        testSequence = new SequentialCommandGroup(
                new AnnounceCommand("--- STARTING HOLONOMIC TEST ---"),
                followHolonomic,
                new AnnounceCommand("--- HOLONOMIC TEST FINISHED ---"),
                new AnnounceCommand("--- STARTING RAMSETE TEST (Robot should drive in a circle) ---"),
                followRamsete,
                new AnnounceCommand("--- RAMSETE TEST FINISHED ---")
        );
    }

    @Override
    protected void onStart() {
        RobotLog.d("onStart called");
        loopTimer.reset();
        schedule(testSequence);
    }

    @Override
    protected void periodic() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        simulatedRobot.update(dt);

        logCounter++;
        if (logCounter % 10 == 0) {
            List<Double> pos = simulatedRobot.getTruePosition();
            RobotLog.i(String.format("SimRobot State | Pos: [x=%.2f, y=%.2f, th=%.2f] | dt: %.4f",
                    pos.get(0), pos.get(1), Math.toDegrees(pos.get(2)), dt));
        }
    }

    @Override
    protected void onStop() {
        RobotLog.d("onStop called");
    }

    private static class AnnounceCommand implements Command {
        private final String message;
        public AnnounceCommand(String message) { this.message = message; }
        @Override
        public void start() { RobotLog.w(message); }
        @Override
        public boolean periodic() { return true; }
    }
}