package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.beaver.optimizedhardware.OptimizedMotor;
import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake implements Subsystem {
    private OptimizedMotor intake;
    private OptimizedMotor stopper;

    private boolean transfer = false;

    private double power = 0;

    public Intake() {
        intake = new OptimizedMotor(HardwareManager.claim(DcMotorEx.class, "intake"), 0.01);
        stopper = new OptimizedMotor(HardwareManager.claim(DcMotorEx.class, "stopper"), 0.01);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        stopper.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intake(boolean intake) {
        if (intake) {
            power = 1;
        } else {
            power = -0.5;
        }
    }

    public void stop() {
        power = 0;
    }

    public void transfer(boolean transfer) {
        this.transfer = transfer;
    }

    public void periodic() {
        intake.setPower(power);

        if (power != 0) {
            stopper.setPower(transfer ? 1 : -1);
        } else {
            stopper.setPower(0);
        }

        CommandOpMode.packet.put("Current", intake.getCurrent(CurrentUnit.AMPS));
    }
}
