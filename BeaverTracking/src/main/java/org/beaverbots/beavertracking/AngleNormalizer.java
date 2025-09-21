package org.beaverbots.beavertracking;

public final class AngleNormalizer {
    private double lastAngleRaw;
    private long rotations;

    private final double period;
    private final double halfPeriod;

    public AngleNormalizer(double initialAngle, double lowerBound, double upperBound) {
        this.period = upperBound - lowerBound;
        this.halfPeriod = this.period / 2;

        double normalizedInitial = initialAngle - lowerBound;
        this.rotations = (long) Math.floor(normalizedInitial / period);
        this.lastAngleRaw = initialAngle - (this.rotations * period);
    }

    public double update(double currentAngleRaw) {
        double delta = currentAngleRaw - lastAngleRaw;

        if (delta > halfPeriod) {
            rotations--;
        } else if (delta < -halfPeriod) {
            rotations++;
        }

        lastAngleRaw = currentAngleRaw;

        return currentAngleRaw + rotations * period;
    }

    public double getAccumulatedAngle() {
        return lastAngleRaw + rotations * period;
    }
}