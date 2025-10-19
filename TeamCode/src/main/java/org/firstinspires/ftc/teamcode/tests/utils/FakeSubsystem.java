package org.firstinspires.ftc.teamcode.tests.utils;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.Subsystem;

import java.util.Set;

public final class FakeSubsystem implements Subsystem {
    final Set<Subsystem> dependencies;
    final String name;

    public FakeSubsystem(Set<Subsystem> dependencies, String name) {
        this.dependencies = dependencies;
        this.name = name;
    }

    @Override
    public Set<Subsystem> getDependencies() {
        RobotLog.d(String.format("FakeSubsystem : %s : getDependencies() called", name));
        return dependencies;
    }

    @Override
    public void periodic() {
        RobotLog.d(String.format("FakeSubsystem : %s : periodic() called", name));
    }

    @Override
    public void periodicDefault() {
        RobotLog.d(String.format("FakeSubsystem : %s : defaultPeriodic() called", name));
    }

    @Override
    public String toString() {
        return name;
    }
}
