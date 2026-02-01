package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pathing.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Modulus;
import org.beaverbots.beaver.util.PiecewiseLinearFunction;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

public final class TurretV2 implements Subsystem {
    private CachedMotor shooterLeft;
    private CachedMotor shooterRight;
    private CachedServo hood;
    private CachedServo turret1;
    private CachedServo turret2;
    private double rpm = 0;
    private double turretAngle = Math.PI;
    private double hoodAngle = Math.PI / 4;

    private PIDFAxis pidf = new PIDFAxis(new PIDFAxis.K(Constants.pidPShooter, Constants.pidIShooter, 0, 1, 0.01, 1, 1, Constants.pidGammaShooter));
    private VoltageSensor voltageSensor;
    private Stopwatch stopwatch;

    public TurretV2(VoltageSensor voltageSensor) {
        shooterLeft = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "shoot"), 0.001);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "shoot2"), 0.001);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = new CachedServo(HardwareManager.claim("hood"), 0.0);

        turret1 = new CachedServo(HardwareManager.claim("turret servo"), 0.001);
        turret2 = new CachedServo(HardwareManager.claim("turret servo2"), 0.001);

        this.voltageSensor = voltageSensor;

        stopwatch = new Stopwatch();
    }


    public void periodic() {
        double control = pidf.update(getError(), rpm * Constants.shooterFrictionConversionFactor / voltageSensor.getVoltage() * 1.3, stopwatch.getDt());
        if (Double.isFinite(control)) {
            shooterLeft.setPower(control);
            shooterRight.setPower(control);
        }

        // https://www.desmos.com/calculator/ptr68yaqmv
        hood.setPosition(2.32825 * (hoodAngle - 0.4988));
        // Invert and add offset to get to pi
        turret1.setPosition(1 - turretAngle / (2 * Math.PI));
        turret2.setPosition(1 - turretAngle / (2 * Math.PI));
    }

    public void setFlywheelSpeed(double rpm) {
        this.rpm = Math.min(rpm, 3200);
    }

    public double getDesiredFlywheelSpeed() {
        return rpm;
    }

    public void setHoodAngle(double angle) {
        if (Double.isFinite(angle))
            hoodAngle = Math.max(Math.toRadians(35), Math.min(angle, Math.toRadians(49)));
    }

    public double getHoodAngle() {
        return hoodAngle;
    }

    public double getHoodPosition() {
        return hood.getPosition();
    }

    public void setTurretAngle(double angle) {
        if (Double.isFinite(angle))
            turretAngle = Math.max(0.25 * 2 * Math.PI, Math.min(Modulus.modulus(angle, 2 * Math.PI), 0.75 * 2 * Math.PI));
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getFlywheelSpeed() {
        double rpm1 = shooterLeft.getVelocity() / 28.0 * 60.0;
        double rpm2 = shooterRight.getVelocity() / 28.0 * 60.0;
        return (rpm1 + rpm2) / 2.0;
    }

    public double getError() {
        return rpm - getFlywheelSpeed();
    }

    ///  Output order: flywheel rpm, hood angle
    public Pair<Double, Double> getSettings(double distance) {
        double rpm = new PiecewiseLinearFunction(List.of(
                new Pair<>(53.62, 1800.0),
                new Pair<>(85.79, 2150.0),
                new Pair<>(105.25, 2300.0),
                new Pair<>(135.99, 2750.0)
        )).evaluate(distance);

        double hood = new PiecewiseLinearFunction(List.of(
                new Pair<>(53.62, 0.6629),
                new Pair<>(85.79, 0.7761),
                new Pair<>(105.25, 0.8338),
                new Pair<>(135.99, 0.8578)
        )).evaluate(distance);

        return new Pair<>(rpm, hood);
    }
}
