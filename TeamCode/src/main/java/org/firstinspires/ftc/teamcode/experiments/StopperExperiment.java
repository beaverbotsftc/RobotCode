package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;

@Autonomous
public class StopperExperiment extends CommandRuntimeOpMode {
    private GamepadEx gamepad;
    private DcMotorEx intake;
    private DcMotorEx stopper;

    public void onInit() {
        gamepad = new GamepadEx(gamepad1);
        intake = HardwareManager.claim("intake");
        stopper = HardwareManager.claim("stopper");

        register(gamepad);
    }

    public void periodic() {
        intake.setPower(gamepad.getLeftY());
        stopper.setPower(gamepad.getRightY());
    }
}
