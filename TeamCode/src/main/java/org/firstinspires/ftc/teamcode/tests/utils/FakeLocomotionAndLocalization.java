package org.firstinspires.ftc.teamcode.tests.utils;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beavertracking.Localizer;
import org.beaverbots.beavertracking.Locomotion;

import java.util.List;

public final class FakeLocomotionAndLocalization implements Locomotion, Localizer {
    public static enum LocomotionType {
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

    @Override
    public void move(List<Double> movement) {
        switch (locomotionType) {
            case HOLONOMIC: {
                double vxDesired = movement.get(0);
                double vyDesired = movement.get(1);
                double vthetaDesired = movement.get(2);

                RobotLog.d("FakeLocomotionAndLocalization : %s : move() called; holonomic mode; desired velocity: %.3f, %.3f, %.3f rad", name, vxDesired, vyDesired, vthetaDesired);

                vx += (vxDesired - vx) * acceleration;
                vy += (vyDesired - vy) * acceleration;
                vtheta += (vthetaDesired - vtheta) * angularAcceleration;

                break;
            }
            case TANK: {
                double vForwardDesired = movement.get(0);
                double vthetaDesired = movement.get(1);

                RobotLog.d(String.format("FakeLocomotionAndLocalization : %s : move() called; tank mode; desired velocity: %.3f, %.3f rad", name, vForwardDesired, vthetaDesired));

                double vxDesired = vForwardDesired * Math.cos(theta);
                double vyDesired = vForwardDesired * Math.sin(theta);

                vx += (vxDesired - vx) * acceleration;
                vy += (vyDesired - vy) * acceleration;
                vtheta += (vthetaDesired - vtheta) * angularAcceleration;

                double forwardSpeed = vx * Math.cos(theta) + vy * Math.sin(theta);
                vx = forwardSpeed * Math.cos(theta);
                vy = forwardSpeed * Math.sin(theta);

                break;
            }
        }

        vx *= 1 - frictionalDeceleration;
        vy *= 1 - frictionalDeceleration;
        vtheta *= 1 - frictionalAngularDeceleration;

        x += vx;
        y += vy;
        theta += vtheta;
    }
}
