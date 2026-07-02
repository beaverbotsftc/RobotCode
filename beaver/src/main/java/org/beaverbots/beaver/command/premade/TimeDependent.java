package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.util.Stopwatch;

import java.util.function.DoubleConsumer;

public class TimeDependent implements Command {
    DoubleConsumer f;
    double time;
    Stopwatch stopwatch;

    public TimeDependent(DoubleConsumer f, double time) {
        this.f = f;
        this.time = time;
        stopwatch = new Stopwatch();
    }

    @Override
    public void start() {
        stopwatch.reset();
    }

    @Override
    public boolean periodic() {
        f.accept(stopwatch.getElapsed());
        return stopwatch.getElapsed() >= time;
    }
}
