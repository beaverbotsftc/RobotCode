package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Geometry;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

public class Turret implements Subsystem {
    private CachedServo turretLeft;
    private CachedServo turretRight;

    private CachedMotor shooterLeft;
    private CachedMotor shooterRight;

    private VoltageSensor voltageSensor;

    private Stopwatch stopwatch;

    private double desiredVelocity = 0;

    private PIDFAxis pidf = new PIDFAxis(new PIDFAxis.K(Constants.pidPShooter, Constants.pidIShooter, 0, 1, 0.5, 1, 1, Constants.pidGammaShooter));

    public Turret(VoltageSensor voltageSensor) {
        turretLeft = new CachedServo(HardwareManager.claim(Servo.class, "left turret"), 0.001);
        turretRight = new CachedServo(HardwareManager.claim(Servo.class, "right turret"), 0.001);

        shooterLeft = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "left shooter"), 0.001);
        shooterRight = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "right shooter"), 0.001);

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.voltageSensor = voltageSensor;

        stopwatch = new Stopwatch();
    }

    public void turn(double angle) {
        double normalized = Math.max(-2 * Math.PI / 3, Math.min(Geometry.normalizeAngle2(-angle), 2 * Math.PI / 3));
        turretLeft.setPosition((normalized + Math.PI) / (2 * Math.PI));
        turretRight.setPosition((normalized + Math.PI) / (2 * Math.PI) - 0.125 / 100);
    }

    public void shoot(double velocity) {
        desiredVelocity = velocity;
    }

    public double getVelocity() {
        return shooterLeft.getVelocity() / 28 * 60;
    }

    public void periodic() {
        double velocity = getVelocity();

        if (desiredVelocity == 0) {
            shooterRight.setPower(0);
            shooterLeft.setPower(0);
            return;
        }

        double control = pidf.update(desiredVelocity - velocity, desiredVelocity * Constants.shooterFrictionConversionFactor / voltageSensor.getVoltage(), stopwatch.getDt());
        if (Double.isFinite(control)) {
            shooterLeft.setPower(control);
            shooterRight.setPower(control);
        }
    }
}
