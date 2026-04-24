package org.beaverbots.beaver.pidf;

import org.beaverbots.beaver.util.LowPassFilter;

public final class PIDFAxis {
    public static class K {
        public final double p;
        public final double i;
        public final double d;
        public final double[] f;

        public final double integrationClamp;
        public final double outputClamp;
        public final double tau;
        public final double gamma;
        public final double maxDt;

        public K(double p, double i, double d, double[] f,
                 double integrationClamp, double outputClamp,
                 double tau, double gamma, double maxDt) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.integrationClamp = integrationClamp;
            this.outputClamp = outputClamp;
            this.tau = tau;
            this.gamma = gamma;
            this.maxDt = maxDt;
        }
    }

    private final K k;

    private double i = 0;
    private double lastError = Double.NaN;

    private final LowPassFilter dLowPassFilter;

    public PIDFAxis(K k) {
        this.k = k;
        dLowPassFilter = new LowPassFilter(k.tau);
    }

    public double update(double error, double[] feedforward, double dt) {
        dt = Math.min(k.maxDt, dt); // Okay because pass-by-value
        if (Double.isNaN(lastError)) lastError = error;
        final double dNoisy = (error - lastError) / dt;
        lastError = error;

        double derivative = dLowPassFilter.update(dNoisy, dt);

        double dampingFactor = 1.0 / (1.0 + k.gamma * (derivative * derivative));

        i += (error * dt) * dampingFactor;

        i = Math.min(Math.max(i, -k.integrationClamp), k.integrationClamp);

        double feedforwardAccumulator = 0;
        for (int index = 0; index < k.f.length; index++)
            feedforwardAccumulator += k.f[index] * feedforward[index];

        double output = (k.p * error) + (k.i * i) + (k.d * derivative) + feedforwardAccumulator;

        return Math.min(Math.max(output, -k.outputClamp), k.outputClamp);
    }

    public void reset() {
        i = 0;
        lastError = Double.NaN;
        dLowPassFilter.reset();
    }
}