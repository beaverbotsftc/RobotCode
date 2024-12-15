package org.firstinspires.ftc.teamcode.pathfollower2;

public class PID {
    public static class K {
        public double p;
        public double i;
        public double d;

        public K(double p, double i, double d) {
            this.p = p;
            this.i = i;
            this.d = d;
        }
    }

    public K k;

    public double i = 0;
    public double d;

    public double lastError;

    public void update(double error, double dt) {
        i += dt * (error - (error - lastError) / 2); // Better integration than just i += dt * error
        d = (error - lastError) / dt;
        lastError = error;
    }

    public double getCorrection() {
        return -(k.p * lastError + k.i * i + k.d * d);
    }

    public PID(K k, double error) {
        this.k = k;
        this.lastError = error;
    }
}
