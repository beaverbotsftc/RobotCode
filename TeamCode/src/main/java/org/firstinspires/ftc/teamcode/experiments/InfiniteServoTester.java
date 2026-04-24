package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.beaverbots.beaver.InfiniteServo;
import org.beaverbots.beaver.optimizedhardware.OptimizedCRServo;
import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;

@TeleOp
public class InfiniteServoTester extends CommandOpMode {
    InfiniteServo s;
    GamepadEx gamepad;

    public void onInit() {
        OptimizedCRServo backLeftServo = new OptimizedCRServo(HardwareManager.claim(CRServo.class, "back left servo"), 0);
        backLeftServo.setPwmRange(500, 2500);
        AnalogInput backLeftEncoder = HardwareManager.get(AnalogInput.class, "back left encoder");
        s = new InfiniteServo(
                backLeftServo,
                backLeftEncoder,
                new PIDFAxis(
                        new PIDFAxis.K(
                                0.2455, 0.1564, 0.0048, new double[] {0}, 1, 1, 0.2281, 7.8312, 0.1
                        )
                ),
                5.948082090796675
        );
        gamepad = new GamepadEx(gamepad1);

        register(gamepad, s);
    }

    public void periodic() {
        packet.put("Theta", gamepad.getRightX());
        packet.put("Real theta", s.getAngle());
        s.setAngle(gamepad.getRightX());
    }
}