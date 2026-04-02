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
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SwerveModule;

import java.util.List;

@TeleOp
public class SwerveModuleExperiment extends CommandRuntimeOpMode {
    GamepadEx gamepad;
    SwerveModule module;

    public void onInit() {
        gamepad = new GamepadEx(gamepad1);
        module = new SwerveModule(new InfiniteServo(new CachedCRServo(HardwareManager.claim(CRServo.class, "swerveservo"), 0), HardwareManager.claim(AnalogInput.class, "swerveinput"),
                new PIDFAxis(new PIDFAxis.K(
                        //0.8 * 0.333, 0.666 * 0.8 / 0.25, 0.25 * 0.8 * 0.111, 0, 1, 1, 0.5, 100
                        //0.8 * 0.2, 0.4 * 0.8 / 0.25, 0.25 * 0.8 * 0.0666, 0, 0, 1, 0.1, 0
                        // Auto-tuned loss: 0.0416
                        0.3239, 0.9773, 0.0001, new double[] {0}, 0, 1, 0.2072, 527.6937))), HardwareManager.claim("front left drive")
        );

        register(gamepad, module);
    }

    public void periodic() {
        module.drive(new Transform(gamepad.getLeftX(), gamepad.getLeftY()));
    }
}
