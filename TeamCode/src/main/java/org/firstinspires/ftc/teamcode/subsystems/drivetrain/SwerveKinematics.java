package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.beaverbots.beaver.util.Transform;

public final class SwerveKinematics {
    public static double getWheelVelocity(Transform wheelPosition, double wheelDirection, Transform chassisTarget) {
        Transform vLinear = chassisTarget.lateral();
        Transform vAngular = new Transform(-chassisTarget.getTheta() * wheelPosition.getY(), chassisTarget.getTheta() * wheelPosition.getX(), 0);
        return vLinear.add(vAngular).dotLateral(Transform.FORWARD.rotateLateral(wheelDirection));
    }
}