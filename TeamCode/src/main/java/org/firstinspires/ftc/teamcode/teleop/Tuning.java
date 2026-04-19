package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.beaver.cachedhardware.CachedMotor;
import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@TeleOp
public class Tuning extends CommandOpMode {
    private VoltageSensor voltageSensor;
    private GamepadEx gamepad;

    private CachedMotor shooterLeft;
    private CachedMotor shooterRight;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        gamepad = new GamepadEx(gamepad1);

        shooterLeft = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "left shooter"), 0.001);
        shooterRight = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "right shooter"), 0.001);

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        register(voltageSensor, gamepad);
    }

    double a = 0;
    double b = 0;

    @Override
    public void periodic() {
        a += (gamepad.getRightY()) * 10;
        b += (gamepad.getRightTrigger() - gamepad.getLeftTrigger()) * 0.01;
        shooterLeft.setPower(a * Math.exp(b) / voltageSensor.getVoltage());
        telemetry.addData("RPM", shooterLeft.getVelocity() * 60 / 28);
        //
        telemetry.addData("DESIRED", a);
        telemetry.addData("B", Math.exp(b) * 100);
    }
}