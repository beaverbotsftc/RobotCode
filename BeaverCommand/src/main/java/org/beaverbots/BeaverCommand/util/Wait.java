package org.beaverbots.BeaverCommand.util;

import org.beaverbots.BeaverCommand.Command;

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
