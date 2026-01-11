package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.pathing.pidf.PIDFAxis;
import org.firstinspires.ftc.teamcode.Constants;

public final class Shooter implements Subsystem {
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private Servo hood;
    private double rpm;
    private PIDFAxis pidf = new PIDFAxis(new PIDFAxis.K(Constants.pidPShooter, Constants.pidIShooter, 0, 1, 0.01, 1, 1, Constants.pidGammaShooter));
    private VoltageSensor voltageSensor;
    private Stopwatch stopwatch;
    private boolean released = false;

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

        double control = pidf.update(getError() , rpm * Constants.shooterFrictionConversionFactor / voltageSensor.getVoltage(),  stopwatch.getDt());
        shooterLeft.setPower(control);
        shooterRight.setPower(control);
    }

    public void spin(double rpm) {
        this.rpm = rpm;
    }

    public void setHood(double pos){
        hood.setPosition(pos);
    }

    public double getHood(){ return hood.getPosition(); }

    public double getVelocity(){
        double rpm1 = shooterLeft.getVelocity() / 28.0 * 60.0;
        double rpm2 = shooterRight.getVelocity() / 28.0 * 60.0;
        return (rpm1 + rpm2) / 2.0;
    }

    public double getError() {
        return rpm - getVelocity();
    }
    
    public Pair<Double, Double> getSettingsAtDistance(double d) { //First value is rpm, second value is hood angle
        return new Pair<>(
                -0.0437804 * d * d + 17.93685 * d + 1225.09339,
                0.00455148 * d + 0.0552841
        );
    }
}
