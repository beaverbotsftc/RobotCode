package org.beaverbots.BeaverCommand.util;

public final class Stopwatch {
    private long start;

    public Stopwatch() {
        start = System.nanoTime();
    }

    public double getElapsed() {
        return (System.nanoTime() - start) * 1e-9;
    }

    public void reset() {
        start = System.nanoTime();
    }
}
