package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.Subsystem;

public final class Led implements Subsystem {

    private static final double PURPLE = 0.721;
    private static final double UNDER_TOLERANCE = 0.01; // 1% under
    private static final double OVER_TOLERANCE  = 0.03; // 3% over

    private Servo rpmLed;
    private Servo ballLed;

    private double power = 0.0;
    private double power2 = 0.0;

    private enum BallState {
        OFF,
        PURPLE
    }

    private final BallState[] ballHistory = new BallState[3];
    private int ballIndex = 0;
    private BallState filteredBallState = BallState.OFF;

    public Led() {
        rpmLed = HardwareManager.claim("led");
        ballLed = HardwareManager.claim("led2");

        // Initialize history to OFF
        for (int i = 0; i < ballHistory.length; i++) {
            ballHistory[i] = BallState.OFF;
        }
    }

    @Override
    public void periodic() {
        rpmLed.setPosition(power);
        ballLed.setPosition(power2);
    }

    public void turnOffRPMLed() {
        power = 0.0;
    }

    public void setLeds(double power) {
        this.power = power;
        this.power2 = power;
    }

    public void setRPMPurple() { power = PURPLE; }

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

    public void turnOffBallLed() {
        updateBallState(BallState.OFF);
    }

    public void setBallLedPurple() {
        updateBallState(BallState.PURPLE);
    }

    private void updateBallState(BallState newState) {
        // Store new state in rolling buffer
        ballHistory[ballIndex] = newState;
        ballIndex = (ballIndex + 1) % ballHistory.length;

        // Majority vote
        int purpleCount = 0;
        int offCount = 0;

        for (BallState state : ballHistory) {
            if (state == BallState.PURPLE) purpleCount++;
            else offCount++;
        }

        BallState majority =
                (purpleCount >= 2) ? BallState.PURPLE : BallState.OFF;

        // Only update output if majority changed
        if (majority != filteredBallState) {
            filteredBallState = majority;
            power2 = (filteredBallState == BallState.PURPLE) ? PURPLE : 0.0;
        }
    }

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}