package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.beaverbots.BeaverCommand.Subsystem;

public interface Drivetrain extends Subsystem {
    void setMotion(double x, double y, double theta);
    void setMaxPower(double maxPower);
}
