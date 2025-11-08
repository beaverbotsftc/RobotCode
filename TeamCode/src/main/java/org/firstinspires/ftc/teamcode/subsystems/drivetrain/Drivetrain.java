package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.beaverbots.BeaverCommand.Subsystem;
import org.beaverbots.beavertracking.Locomotion;
import org.firstinspires.ftc.teamcode.DrivetrainState;

import java.util.List;

public interface Drivetrain extends Locomotion {
    void move(DrivetrainState velocity);
    void move(List<Double> velocity);
    void setMaxPower(double maxPower);
}
