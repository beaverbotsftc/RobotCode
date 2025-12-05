package org.beaverbots.beavertracking;

public final class PIDFAxis {
    public static class K {
        public final double p;
        public final double i;
        public final double d;
        public final double f;

        public final double integrationClamp;
        public final double outputClamp;
        public final double tau;

        public K(double p, double i, double d, double f, double integrationClamp, double outputClamp, double tau) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.integrationClamp = integrationClamp;
            this.outputClamp = outputClamp;
            this.tau = tau;
        }
    }

    private final K k;

    private double i = 0;
    private double lastError = 0;

    private final LowPassFilter dLowPassFilter;

    public PIDFAxis(K k) {
        this.k = k;
        dLowPassFilter = new LowPassFilter(k.tau);
    }

    public double update(double error, double feedforward, double dt) {
        final double dNoisy = (error - lastError) / dt;
        lastError = error;
        double derivative = dLowPassFilter.update(dNoisy, dt);

        if (Math.abs(unclampedControl(error, derivative, feedforward)) < k.outputClamp) {
            i += error * dt;
            i = Math.min(Math.max(i, -k.integrationClamp), k.integrationClamp);
        }

        return Math.min(Math.max(unclampedControl(error, derivative, feedforward), -k.outputClamp), k.outputClamp);
    }

    private double unclampedControl(double error, double derivative, double feedforward) {
        return k.p * error + k.i * i + k.d * derivative + k.f * feedforward;
    }
}
