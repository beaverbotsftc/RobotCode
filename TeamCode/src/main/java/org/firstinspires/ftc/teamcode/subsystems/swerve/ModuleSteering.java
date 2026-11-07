package org.firstinspires.ftc.teamcode.subsystems.swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.beaver.cachedhardware.CachedCRServo;
import org.firstinspires.ftc.teamcode.beaver.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.beaver.command.HardwareManager;
import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.beaver.util.Beaver;
import org.firstinspires.ftc.teamcode.beaver.util.DoubleRing;

public class ModuleSteering implements Subsystem {
    private static final String[] SERVO_NAMES =   {"front left swervo",         "front right swervo",         "back left swervo",         "back right swervo"        };
    private static final String[] ENCODER_NAMES = {"front left swervo encoder", "front right swervo encoder", "back left swervo encoder", "back right swervo encoder"};

    private static final double[] ENCODER_OFFSETS = {0, 0, 0, 0};
    private static final double[] POWER_COEFFICIENTS = {1, 1, 1, 1};
    private static final double[] LATENCY = {0.1, 0.1, 0.1, 0.1};
    private static final double[] POWER_TOLERANCE = {0.01, 0.01, 0.01, 0.01};

    private final CachedCRServo servo;
    private final AnalogInput encoder;

    private final int id;

    private DoubleRing encoderValues;
    private DoubleRing velocities;
    private DoubleRing times;


    public ModuleSteering(int id) {
        this.id = id;

        servo = new CachedCRServo(HardwareManager.claim(SERVO_NAMES[id]), POWER_TOLERANCE[id]);
        encoder = HardwareManager.claim(ENCODER_NAMES[id]);

        int history = (int) Math.ceil(2 * LATENCY[id] / CommandOpMode.getAverageDt());
        encoderValues = new DoubleRing(history);
        velocities = new DoubleRing(history);
        times = new DoubleRing(history);
    }

    public double getPosition() {
        // TODO: Perhaps use an interpolated lookup table here
        return Beaver.modulus(
                encoderValues.get(-1) * 2 * Math.PI + ENCODER_OFFSETS[id]
                        + getVelocityPast(CommandOpMode.getElapsed() - LATENCY[id]) * LATENCY[id],
                2 * Math.PI
        );
    }

    public void setVelocity(double v) {
        // TODO: Voltage correction
        servo.setPower(v * POWER_COEFFICIENTS[id]);
        velocities.set(-1, v);
    }

    public double getVelocityPast(double time) {
        int index = times.binarySearch(time);

        double before = times.get(index);
        double after = times.get(index + 1);
        if (index == times.getStoredCount() - 1) return velocities.get(index);

        double t = (time - before) / (after - before);

        return velocities.get(index + 1) * t + velocities.get(index) * (1 - t);
    }


    public void periodic() {
        encoderValues.append(encoder.getVoltage() / encoder.getMaxVoltage());
        velocities.append(velocities.get(-1));
        times.append(CommandOpMode.getElapsed());
    }
}
