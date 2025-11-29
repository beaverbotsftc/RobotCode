package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.beaverbots.beavertracking.Locomotion;

import java.util.List;

public interface Drivetrain extends Locomotion {
    void move(DrivetrainState velocity);
    void move(DrivetrainState velocity, DrivetrainState position);
    void move(List<Double> velocity, List<Double> position);
    void setMaxPower(double maxPower);
}
