package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.beaverbots.beaver.pathing.Locomotion;
import org.beaverbots.beaver.util.Transform;

import java.util.List;

public interface Drivetrain extends Locomotion {
    void move(Transform force);
    void move(Transform force, Transform position);
    void move(List<Double> force, List<Double> position);
}
