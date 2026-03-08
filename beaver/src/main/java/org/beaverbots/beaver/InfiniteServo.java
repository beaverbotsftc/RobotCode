package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.beaverbots.beaver.cachedhardware.CachedCRServo;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;

import java.util.List;

public class InfiniteServo implements Subsystem {
    private PIDF pidf = new PIDF(List.of(new PIDFAxis(new PIDFAxis.K(
            // Auto-tuned loss: 0.0416
            0.3239, 0.9773, 0.0001, 0, 0, 1, 0.2072, 527.6937
    ))));

    private CachedCRServo servo;
    private AnalogInput encoder;


    public InfiniteServo(String servoId, String encoderId, double delta) {
        servo = new CachedCRServo(HardwareManager.claim(CRServo.class, "servo"), delta);
        encoder = HardwareManager.claim(AnalogInput.class, "encoder");
    }

    public double getAngle() {
        return encoder.getVoltage() / encoder.getMaxVoltage() * 2 * Math.PI;
    }

    public void setAngle(double angle) {
    }
}
