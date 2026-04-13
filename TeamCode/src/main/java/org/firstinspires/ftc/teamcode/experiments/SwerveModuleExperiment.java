package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.beaverbots.beaver.InfiniteServo;
import org.beaverbots.beaver.cachedhardware.CachedCRServo;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SwerveModule;

import java.util.List;

@TeleOp
public class SwerveModuleExperiment extends CommandRuntimeOpMode {
    GamepadEx gamepad;
    SwerveModule module;

    public void onInit() {
        VoltageSensor voltageSensor = new VoltageSensor();
        gamepad = new GamepadEx(gamepad1);
        module = new SwerveModule(new InfiniteServo(new CachedCRServo(HardwareManager.claim(CRServo.class, "front left servo"), 0), HardwareManager.claim(AnalogInput.class, "front left servo encoder"),
                new PIDFAxis(new PIDFAxis.K(
                        // Auto-tuned loss: 0.087
                        0.2455, 0.1564, 0.0048, new double[] {0}, 1, 1, 0.2281, 7.8312
                ))), HardwareManager.claim("front left drive"), voltageSensor,
        new Transform(1, 1));

        register(voltageSensor, gamepad, module);
    }

    public void periodic() {
        module.drive(new Transform(gamepad.getLeftX(), gamepad.getLeftY()));
    }
}
