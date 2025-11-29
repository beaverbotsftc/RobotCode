package org.firstinspires.ftc.teamcode.tests.utils;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beavertracking.Localizer;
import org.beaverbots.beavertracking.Locomotion;

import java.util.List;

public final class FakeLocomotionAndLocalization implements Locomotion, Localizer {
    public enum LocomotionType {
        HOLONOMIC,
        TANK,
    }

    final LocomotionType locomotionType;

    final double acceleration;
    final double angularAcceleration;

    final double frictionalDeceleration;
    final double frictionalAngularDeceleration;

    final String name;

    double x;
    double y;
    double theta;

    double vx = 0;
    double vy = 0;
    double vtheta = 0;


    public FakeLocomotionAndLocalization(LocomotionType locomotionType, double acceleration, double angularAcceleration, double frictionalDeceleration, double frictionalAngularDeceleration, double x, double y, double theta, String name) {
        this.locomotionType = locomotionType;
        this.acceleration = acceleration;
        this.angularAcceleration = angularAcceleration;
        this.frictionalDeceleration = frictionalDeceleration;
        this.frictionalAngularDeceleration = frictionalAngularDeceleration;
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.name = name;
    }

    @Override
    public List<Double> getPositionAsList() {
        RobotLog.d(String.format("FakeLocomotionAndLocalization : %s : getPosition() called; position: %.3f, %.3f, %.3f rad", name, x, y, theta));
        return List.of(x, y, theta);
    }

    @Override
    public List<Double> getVelocityAsList() {
        RobotLog.d(String.format("FakeLocomotionAndLocalization : %s : getVelocity() called; velocity: %.3f, %.3f, %.3f rad", name, vx, vy, vtheta));
        return List.of(vx, vy, vtheta);
    }

    /**
     * Updates the robot's state based on a desired global velocity command.
     * The simulation assumes each call represents one discrete time step.
     * @param velocity The desired global velocity vector [vx, vy, vtheta].
     * @param position The robot's current global position vector [x, y, theta], used for non-holonomic constraints.
     */
    @Override
    public void move(List<Double> velocity, List<Double> position) {
        double desiredGlobalVx = velocity.get(0);
        double desiredGlobalVy = velocity.get(1);
        double desiredGlobalVtheta = velocity.get(2);
        double currentGlobalTheta = position.get(2);

        switch (locomotionType) {
            case HOLONOMIC: {
                // For a holonomic drive, the desired global velocities can be targeted directly.
                RobotLog.d("FakeLocomotionAndLocalization : %s : move() called; holonomic mode; desired global velocity: %.3f, %.3f, %.3f rad", name, desiredGlobalVx, desiredGlobalVy, desiredGlobalVtheta);

                vx += (desiredGlobalVx - vx) * acceleration;
                vy += (desiredGlobalVy - vy) * acceleration;
                vtheta += (desiredGlobalVtheta - vtheta) * angularAcceleration;
                break;
            }
            case TANK: {
                // For a tank drive, we must enforce non-holonomic constraints. The robot can only move
                // forward along its current heading.
                RobotLog.d("FakeLocomotionAndLocalization : %s : move() called; tank mode; desired global velocity: %.3f, %.3f, %.3f rad", name, desiredGlobalVx, desiredGlobalVy, desiredGlobalVtheta);

                // Project the desired global velocity vector onto the robot's current heading vector
                // to find the desired forward speed.
                double cosTheta = Math.cos(currentGlobalTheta);
                double sinTheta = Math.sin(currentGlobalTheta);
                double desiredForwardSpeed = desiredGlobalVx * cosTheta + desiredGlobalVy * sinTheta;

                // The desired global velocity is now constrained to be only in the forward direction.
                double constrainedVx = desiredForwardSpeed * cosTheta;
                double constrainedVy = desiredForwardSpeed * sinTheta;

                // Update internal velocities towards the constrained target.
                vx += (constrainedVx - vx) * acceleration;
                vy += (constrainedVy - vy) * acceleration;
                vtheta += (desiredGlobalVtheta - vtheta) * angularAcceleration;
                break;
            }
        }

        // Apply friction to the actual velocities
        vx *= (1 - frictionalDeceleration);
        vy *= (1 - frictionalDeceleration);
        vtheta *= (1 - frictionalAngularDeceleration);

        // Integrate velocity to update position (assuming dt=1 for simplicity)
        x += vx;
        y += vy;
        theta += vtheta;
    }
}