package org.firstinspires.ftc.teamcode.beaverbotv0.utils;

public final class PID {
    public static class K {
        public double proportional;
        public double integral;
        public double derivative;

        public K(double proportional, double integral, double derivative) {
            this.proportional = proportional;
            this.integral = integral;
            this.derivative = derivative;
        }
    }

    private K k;

    private double integralAccumulator = 0;

    private double lastError;

    public double correction(double error, double dt) {
        integralAccumulator += dt * (error - (error - lastError) / 2); // Better integration than just i += dt * error
        final double correction = k.proportional * error + k.integral * integralAccumulator + k.derivative * (error - lastError) / dt;
        lastError = error;
        return correction;
    }

    public PID(K k, double error) {
        this.k = k;
        this.lastError = error;
    }
}
