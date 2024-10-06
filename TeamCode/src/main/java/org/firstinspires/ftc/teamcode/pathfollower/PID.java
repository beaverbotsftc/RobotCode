package org.firstinspires.ftc.teamcode.pathfollower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID {
    private double lastError;
    private double kP;
    private double kI;
    private double kD;
    private double partialIntegration = 0;

    public double updateAndGetCorrection(double error, double dt, Telemetry telemetry) {
        double dError = (error - lastError) / dt;
        telemetry.addData("dt", dt);
        telemetry.addData("error - lastError", error - lastError);
        telemetry.addData("dError", dError);
        this.partialIntegration += error * dt;
        telemetry.addData("error", error);
        telemetry.addData("error * dt", error * dt);
        telemetry.addData("partialInt", this.partialIntegration);
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
