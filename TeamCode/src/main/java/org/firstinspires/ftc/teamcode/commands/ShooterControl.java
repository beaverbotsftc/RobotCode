package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverOptimize.IteratedLocalSearch;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterControl implements Command {
    Gamepad gamepad;
    Shooter shooter;

    double shootRpm = 2050.0;

    public ShooterControl(Shooter shooter, Gamepad gamepad) {
        this.shooter = shooter;
        this.gamepad = gamepad;
    }

    public boolean periodic() {
        if (gamepad.getDpadLeft()) {
            shooter.spin(shootRpm);

            double velocity = shooter.getVelocity();

            if (Math.abs(velocity - shootRpm) / shootRpm <= 0.05) {
                gamepad.rumble(0.45, 0.45, com.qualcomm.robotcore.hardware.Gamepad.RUMBLE_DURATION_CONTINUOUS);
            } else {
                gamepad.stopRumble();
            }
        } else if (gamepad.getDpadRight()) {
            shooter.spin(-1000);
        } else {
            shooter.spin(0);
        }

        if (gamepad.getTriangle() && shootRpm < 4000) {
            shootRpm += 5;
        }
        if (gamepad.getCross() && shootRpm > 100) {
            shootRpm -= 5;
        }

        if (gamepad.getLeftBumperJustPressed()) {
            shooter.setHood(0.05);
            shootRpm = 2050.0;
        } else if (gamepad.getRightBumperJustPressed()) {
            shooter.setHood(0.51);
            shootRpm = 3000.0;
        }

        return false;
    }

    public double getShootRpm() {
        return shootRpm;
    }
}
