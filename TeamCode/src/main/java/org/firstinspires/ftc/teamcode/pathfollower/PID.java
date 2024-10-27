package org.firstinspires.ftc.teamcode.pathfollower;

public final class PID {
    private double lastError;
    private double kP;
    private double kI;
    private double kD;
    private double partialIntegration = 0;

    public void update(double error, double dt) {
        this.partialIntegration += error * dt;
        lastError = error;
    }

    public double correction(double error, double dt) {
        double dError = (error - lastError) / dt;
        return kP * error + kI * this.partialIntegration + kD * dError;
    }

    public PID(double error, double kP, double kI, double kD) {
        this.lastError = error;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
