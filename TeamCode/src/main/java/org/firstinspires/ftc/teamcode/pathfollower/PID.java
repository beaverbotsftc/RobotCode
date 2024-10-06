package org.firstinspires.ftc.teamcode.pathfollower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID {
    private double lastError;
    private double kP;
    private double kI;
    private double kD;
    private double partialIntegration = 0;

    public double updateAndGetCorrection(double error, double dt) {
        double dError = (error - lastError) / dt;
        this.partialIntegration += error * dt;
        double correction = kP * error + kI * this.partialIntegration + kD * dError;
        lastError = error;
        return correction;
    }

    public PID(double error, double kP, double kI, double kD) {
        this.lastError = error;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
