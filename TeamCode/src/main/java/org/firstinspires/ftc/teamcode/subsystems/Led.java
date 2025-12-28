package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public final class Led implements Subsystem {

    private static final double PURPLE = 0.721;
    private static final double UNDER_TOLERANCE = 0.01; // 1% under
    private static final double OVER_TOLERANCE  = 0.03; // 3% over

    private Servo led;
    private Servo led2;

    private double power = 0.0;

    public Led() {
        led = HardwareManager.claim("led");
        led2 = HardwareManager.claim("led2");
    }

    @Override
    public void periodic() {
        led.setPosition(power);
        led2.setPosition(power);
    }

    public void turnOff() { power = 0.0; }

    public void setLed(double power) { this.power = clamp(power); }

    public void setPurple() { power = PURPLE; }

    public void setProximity(double goal, double current) {
        // Below 50% → LEDs off
        if (current < 0.5 * goal) {
            power = 0.0;
            return;
        }

        double errorPercent = (current - goal) / goal;

        // Lock to purple within asymmetric tolerance band
        if (errorPercent >= -UNDER_TOLERANCE && errorPercent <= OVER_TOLERANCE) {
            power = PURPLE;
            return;
        }

        // Normalize 50% → 100% into 0 → 1
        double normalized = (current / goal - 0.5) / 0.5;
        normalized = clamp(normalized);

        // Below goal → ramp up toward purple
        if (errorPercent < 0) {
            power = PURPLE * normalized;
        }
        // Above goal → only brighten AFTER +3%
        else {
            double overshoot = (errorPercent - OVER_TOLERANCE) / OVER_TOLERANCE;
            overshoot = clamp(overshoot);
            power = PURPLE + (1.0 - PURPLE) * overshoot;
        }

        power = clamp(power);
    }

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}