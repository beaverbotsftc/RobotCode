package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pathing.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Modulus;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.Constants;

public final class TurretV2 implements Subsystem {
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private Servo hood;
    private Servo turret1;
    private Servo turret2;
    private double rpm = 0;
    private double turretAngle = 0.5;
    private double hoodAngle = 0;

    private PIDFAxis pidf = new PIDFAxis(new PIDFAxis.K(Constants.pidPShooter, Constants.pidIShooter, 0, 1, 0.01, 1, 1, Constants.pidGammaShooter));
    private VoltageSensor voltageSensor;
    private Stopwatch stopwatch;

    public TurretV2(VoltageSensor voltageSensor) {
        shooterLeft = HardwareManager.claim(DcMotorEx.class, "shoot");
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight = HardwareManager.claim(DcMotorEx.class, "shoot2");
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = HardwareManager.claim("hood");

        turret1 = HardwareManager.claim("turret servo");
        turret2 = HardwareManager.claim("turret servo2");

        this.voltageSensor = voltageSensor;

        stopwatch = new Stopwatch();
    }


    public void periodic() {
        double control = pidf.update(getError(), rpm * Constants.shooterFrictionConversionFactor / voltageSensor.getVoltage(), stopwatch.getDt());
        if (Double.isFinite(control)) {
            shooterLeft.setPower(control);
            shooterRight.setPower(control);
        }

        // https://www.desmos.com/calculator/ptr68yaqmv
        hood.setPosition(2.26957 * (hoodAngle - 0.463961));
        turret1.setPosition(turretAngle / (2 * Math.PI));
        turret2.setPosition(turretAngle / (2 * Math.PI));
    }

    public void spin(double rpm) {
        this.rpm = Math.min(rpm, 3200);
    }

    public double getDesiredRpm() {
        return rpm;
    }

    public void setHoodAngle(double angle) {
        if (Double.isFinite(angle))
            hoodAngle = Math.max(Math.toRadians(34), Math.min(angle, Math.toRadians(49)));
    }

    public double getHoodAngle() {
        return hoodAngle;
    }

    public void setTurretAngle(double angle) {
        if (Double.isFinite(angle))
            turretAngle = Math.max(0.25 * 2 * Math.PI, Math.min(Modulus.modulus(angle, 2 * Math.PI), 0.75 * 2 * Math.PI));
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getVelocity() {
        double rpm1 = shooterLeft.getVelocity() / 28.0 * 60.0;
        double rpm2 = shooterRight.getVelocity() / 28.0 * 60.0;
        return (rpm1 + rpm2) / 2.0;
    }

    public double getError() {
        return rpm - getVelocity();
    }
}
