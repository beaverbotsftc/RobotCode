package org.firstinspires.ftc.teamcode.tests.utils;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;

import java.util.Set;

public final class FakeCommand implements Command {
    final int finishingTime;
    final Set<Subsystem> dependencies;
    final String name;

    int time;

    public FakeCommand(int time, Set<Subsystem> dependencies, String name) {
        this.finishingTime = time;
        this.dependencies = dependencies;
        this.name = name;
    }

    @Override
    public Set<Subsystem> getDependencies() {
        RobotLog.d(String.format("FakeCommand : %s : getDependencies() called", name));
        return dependencies;
    }

    @Override
    public boolean periodic() {
        time++;
        RobotLog.d(String.format("FakeCommand : %s : periodic() called; time passed: %d; finishing time: %d", name, time, finishingTime));
        return time == finishingTime;
    }

    @Override
    public void start() {
        time = 0;
        RobotLog.d(String.format("FakeCommand : %s : start() called", name));
    }

    @Override
    public void stop() {
        RobotLog.d(String.format("FakeCommand : %s : stop() called", name));
    }

    @Override
    public String toString() {
        return name;
    }
}
