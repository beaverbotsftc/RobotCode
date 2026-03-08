package org.beaverbots.beaver;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.util.Geometry;
import org.beaverbots.beaver.util.Stopwatch;


public class InfiniteServo implements Subsystem {
    private CRServo servo;
    private AnalogInput encoder;
    private PIDF pidf;
    private double offset;

    private Stopwatch stopwatch;
    private double target = 0;


    public InfiniteServo(CRServo servo, AnalogInput encoder, PIDF pidf) {
        this(servo, encoder, pidf, 0);
    }

    public InfiniteServo(CRServo servo, AnalogInput encoder, PIDF pidf, double offset) {
        this.servo = servo;
        this.encoder = encoder;
        this.pidf = pidf;
        this.offset = offset;

        stopwatch = new Stopwatch();
    }

    public double getAngle() {
        return Geometry.normalizeAngle(encoder.getVoltage() / encoder.getMaxVoltage() * 2 * Math.PI + offset);
    }

    public void setAngle(double angle) {
        target = angle;
    }

    public void periodic() {
        double error = Geometry.shortestAngle(getAngle(), target);
        double control = pidf.update(Collections.singletonList(error), Collections.singletonList(0.0), stopwatch.getDt()).get(0);

        servo.setPower(control);
    }
}