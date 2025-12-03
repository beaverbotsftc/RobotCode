package org.beaverbots.BeaverCommand.util;

public final class Stopwatch {
    private long start;
    private double lastTime;

    public Stopwatch() {
        reset();
    }

    public double getElapsed() {
        return (System.nanoTime() - start) * 1e-9;
    }

    public double getDt() {
        double time = getElapsed();
        double dt = time - lastTime;
        lastTime = time;
        return dt;
    }

    public void reset() {
        start = System.nanoTime();
        lastTime = 0;
    }
}
