package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.util.PiecewiseLinearFunction;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.pathing.pidf.PIDFAxis;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

public final class Shooter implements Subsystem {
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private Servo hood;
    private double rpm;
    private PIDFAxis pidf = new PIDFAxis(new PIDFAxis.K(Constants.pidPShooter, Constants.pidIShooter, 0, 1, 0.01, 1, 1, Constants.pidGammaShooter));
    private VoltageSensor voltageSensor;
    private Stopwatch stopwatch;

    public boolean hardStopSetting = false;

    public Shooter(VoltageSensor voltageSensor) {
        shooterLeft = HardwareManager.claim(DcMotorEx.class, "shoot");
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight = HardwareManager.claim(DcMotorEx.class, "shoot2");
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = HardwareManager.claim("hood");

        this.voltageSensor = voltageSensor;

        stopwatch = new Stopwatch();
    }


    public void periodic() {
        if (rpm == 0 && !hardStopSetting) {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
            return;
        }

        if (hardStopSetting) rpm = 0;

        double control = pidf.update(getError(), rpm * Constants.shooterFrictionConversionFactor / voltageSensor.getVoltage(), stopwatch.getDt());
        if (Double.isFinite(control)) {
            shooterLeft.setPower(control);
            shooterRight.setPower(control);
        }
    }

    public void spin(double rpm) {
        this.rpm = Math.min(rpm, 3200);
    }

    public void setHood(double pos) {
        if (Double.isFinite(pos))
            hood.setPosition(Math.min(pos, 1));
    }

    public double getHood() {
        return hood.getPosition();
    }

    public double getVelocity() {
        double rpm1 = shooterLeft.getVelocity() / 28.0 * 60.0;
        double rpm2 = shooterRight.getVelocity() / 28.0 * 60.0;
        return (rpm1 + rpm2) / 2.0;
    }

    public double getError() {
        return rpm - getVelocity();
    }


    public static Pair<Double, Double> getSettingsAtDistance(double d) { //First value is rpm, second value is hood angle
        /*
        double rpm = new PiecewiseLinearFunction(List.of(
                new Pair<>(65.3, 2200.0),
                new Pair<>(79.5, 2375.0),
                new Pair<>(93.6, 2550.0),
                new Pair<>(111.4, 2675.0),
                new Pair<>(120.1, 2725.0),
                new Pair<>(152.6, 2950.0)
        )).evaluate(d);

        double hood = new PiecewiseLinearFunction(List.of(
                new Pair<>(65.3, 0.3),
                new Pair<>(79.5, 0.435),
                new Pair<>(93.6, 0.525),
                new Pair<>(111.4, 0.565),
                new Pair<>(120.1, 0.62),
                new Pair<>(152.6, 0.72)
        )).evaluate(d);
         */
        /*

        double rpm = new PiecewiseLinearFunction(List.of(
                new Pair<>(48.58, 2150.0),
                new Pair<>(62.51, 2450.0),
                new Pair<>(76.46, 2475.0),
                new Pair<>(90.68, 2600.0),
                new Pair<>(101.04, 2675.0),
                new Pair<>(123.07, 2800.0),
                new Pair<>(147.34, 3200.0)
        )).evaluate(d);

        double hood = new PiecewiseLinearFunction(List.of(
                new Pair<>(48.58, 0.32),
                new Pair<>(62.51, 0.56),
                new Pair<>(76.46, 0.61),
                new Pair<>(90.68, 0.62),
                new Pair<>(101.04, 0.66),
                new Pair<>(123.07, 0.675),
                new Pair<>(147.34, 0.9)
        )).evaluate(d);
         */

        double rpm = new PiecewiseLinearFunction(List.of(
                new Pair<>(48.58, 2150.0),
                new Pair<>(62.51, 2450.0),
                new Pair<>(76.46, 2475.0),
                new Pair<>(90.68, 2600.0),
                new Pair<>(101.04, 2675.0),
                new Pair<>(123.07, 2800.0),
                new Pair<>(147.34, 3200.0)
        )).evaluate(d);

        double hood = new PiecewiseLinearFunction(List.of(
                new Pair<>(48.58, 0.32),
                new Pair<>(62.51, 0.56),
                new Pair<>(76.46, 0.61),
                new Pair<>(90.68, 0.62),
                new Pair<>(101.04, 0.66),
                new Pair<>(123.07, 0.675 + 0.05),
                new Pair<>(147.34, 0.9 + 0.05)
        )).evaluate(d);

        return new Pair<>(rpm, hood);
    }
}
