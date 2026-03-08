package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.cachedhardware.CachedServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public class Intake implements Subsystem {
    private CachedMotor intakeLeft;
    private CachedMotor intakeRight;

    private double power = 0;

    public Intake() {
        intakeLeft = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "intake left"), 0.01);
        intakeRight = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "intake right"), 0.01);

        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intake() {
        power = 0.2;
    }

    public void outtake() {
        power = 0.2;
    }

    public void stop() {
        power = 0;
    }

    public void periodic() {
        intakeLeft.setPower(power);
    }
}
