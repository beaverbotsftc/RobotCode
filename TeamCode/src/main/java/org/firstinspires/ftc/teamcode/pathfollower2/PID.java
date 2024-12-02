package org.firstinspires.ftc.teamcode.pathfollower2;


public class PID {
    public static class PIDCoefficients {
        public double p;
        public double i;
        public double d;

        public PIDCoefficients(double p, double i, double d) {
            this.p = p;
            this.i = i;
            this.d = d;
        }
    }
    public PIDCoefficients k;
    public double lastError;
    public double i = 0;

    public double correction;

    public void update(double error, double dt) {
        i += error * dt;
        correction =  k.p * error + k.i * i + k.d * (error - lastError) / dt;
    }

    public PID(double error, PIDCoefficients k) {
        this.lastError = error;
        this.k = k;
    }
}
