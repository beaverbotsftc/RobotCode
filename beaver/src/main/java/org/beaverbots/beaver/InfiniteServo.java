package org.beaverbots.beaver;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Geometry;
import org.beaverbots.beaver.util.Stopwatch;


public class InfiniteServo implements Subsystem {
    private CRServo servo;
    private AnalogInput encoder;
    private PIDFAxis pidf;
    private double offset;

    private Stopwatch stopwatch;
    private double target = 0;


    public InfiniteServo(CRServo servo, AnalogInput encoder, PIDFAxis pidf) {
        this(servo, encoder, pidf, 0);
    }

    public InfiniteServo(CRServo servo, AnalogInput encoder, PIDFAxis pidf, double offset) {
        this.servo = servo;
        this.encoder = encoder;
        this.pidf = pidf;
        this.offset = offset;
        servo.setPower(0); // Fix a dumb bug where you *need* to set the servo power before you can read the position

        stopwatch = new Stopwatch();
    }

    public double getAngle() {
        // Negated because CCW is positive
        return Geometry.normalizeAngle(-encoder.getVoltage() / encoder.getMaxVoltage() * 2 * Math.PI - offset);
    }

    public void setAngle(double angle) {
        target = angle;
    }

    public void periodic() {
        double error = Geometry.shortestAngle(getAngle(), target);
        double control = pidf.update(error, new double[] {0.0}, stopwatch.getDt());

        // Negated because CCW is positive
        servo.setPower(-control);
    }
}