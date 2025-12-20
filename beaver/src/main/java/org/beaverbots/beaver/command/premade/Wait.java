package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.util.Stopwatch;

public class Wait implements Command {
    private double time;
    private Stopwatch stopwatch;

    public Wait(double time) {
        this.time = time;
    }

    public void start() {
        stopwatch = new Stopwatch();
    }

    public boolean periodic() {
        return stopwatch.getElapsed() >= time;
    }
}
