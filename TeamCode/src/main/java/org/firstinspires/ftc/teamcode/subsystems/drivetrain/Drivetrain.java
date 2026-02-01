package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.beaverbots.beaver.pathing.Locomotion;
import org.firstinspires.ftc.teamcode.Transform;

import java.util.List;

public interface Drivetrain extends Locomotion {
    void setBrake(boolean brake);
    void toggleBrake();
    void move(Transform velocity);
    void move(Transform velocity, Transform position);
    void move(List<Double> velocity, List<Double> position);
    void setMaxPower(double maxPower);
}
