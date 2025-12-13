package org.beaverbots.BeaverSensor;

public final class LowPassFilter {
    private final double tau;
    private double yPrevious = 0;

    private double alpha(double dt) {
        return dt / (tau + dt);
    }

    public LowPassFilter(double tau) {
        this.tau = tau;
    }

    public double update(double y, double dt) {
        final double alpha = alpha(dt);
        yPrevious = alpha * y + (1 - alpha) * yPrevious;
        return yPrevious;
    }
}
