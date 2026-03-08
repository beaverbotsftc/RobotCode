package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.beaverbots.beaver.util.Transform;

public final class SwerveKinematics {
    public static double getWheelVelocity(Transform wheelPosition, double wheelDirection, Transform chassisTarget) {
        Transform vLinear = chassisTarget.lateral();
        Transform vAngular = chassisTarget.angular().cross(wheelPosition);
        return vLinear.add(vAngular).lateralDot(Transform.FORWARD.rotate(wheelDirection));
    }
}