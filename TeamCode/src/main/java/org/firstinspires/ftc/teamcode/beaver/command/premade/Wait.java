package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;
import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.beaver.util.Stopwatch;

import java.util.Collections;
import java.util.Set;

public class Wait implements Command {
    private final double time;
    private Stopwatch stopwatch;

    public Wait(double time) {
        this.time = time;
    }

    public void start() {
        stopwatch = new Stopwatch();
    }

    @Override
    public Set<Subsystem> getImmediateDependencies() { return Collections.emptySet(); }

    public boolean periodic() {
        return stopwatch.getElapsed() >= time;
    }
}
