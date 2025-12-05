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
        public final double gamma;

        public K(double p, double i, double d, double f,
                 double integrationClamp, double outputClamp,
                 double tau, double gamma) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.integrationClamp = integrationClamp;
            this.outputClamp = outputClamp;
            this.tau = tau;
            this.gamma = gamma;
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

        double dampingFactor = 1.0 / (1.0 + k.gamma * (derivative * derivative));

        i += (error * dt) * dampingFactor;

        i = Math.min(Math.max(i, -k.integrationClamp), k.integrationClamp);

        double output = (k.p * error) + (k.i * i) + (k.d * derivative) + (k.f * feedforward);

        return Math.min(Math.max(output, -k.outputClamp), k.outputClamp);
    }
}
