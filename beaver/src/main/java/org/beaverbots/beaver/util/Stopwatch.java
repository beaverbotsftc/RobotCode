package org.beaverbots.beaver.util;

public final class Stopwatch {
    private long start;
    private double lastTime;
    private double lastLastTime;

    public Stopwatch() {
        reset();
    }

    public double getElapsed() {
        return (System.nanoTime() - start) * 1e-9;
    }

    public double getDt() {
        double time = getElapsed();
        double dt = time - lastTime;
        lastLastTime = lastTime;
        lastTime = time;
        return dt;
    }

    ///  Calling this twice in a row is undefined.
    public void undoGetDt() {
        lastTime = lastLastTime;
    }

    public void reset() {
        start = System.nanoTime();
        lastTime = 0;
    }
}
