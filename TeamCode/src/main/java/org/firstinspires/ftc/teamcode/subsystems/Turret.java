package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public class Turret implements Subsystem {
    private CachedServo turretLeft;
    private CachedServo turretRight;

    private CachedMotor shooterLeft;
    private CachedMotor shooterRight;

    public Turret() {
        turretLeft = new CachedServo(HardwareManager.claim(Servo.class, "left turret"), 0.001);
        turretRight = new CachedServo(HardwareManager.claim(Servo.class, "right turret"), 0.001);

        shooterLeft = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "left shooter"), 0.001);
        shooterRight = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "right shooter"), 0.001);

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turn(double angle) {
        turretLeft.setPosition(angle / (2 * Math.PI));
        turretRight.setPosition(angle / (2 * Math.PI) - 0.125 / 100);
    }

    public void shoot(double rpm) {
        shooterLeft.setPower(rpm / 6000);
    }
}
