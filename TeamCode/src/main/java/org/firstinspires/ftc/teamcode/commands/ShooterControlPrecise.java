package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.beaver.command.Command;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterControlPrecise implements Command {
    Gamepad gamepad;
    Shooter shooter;

    double shootRpm = 2400;
    double hoodPos = 0.5;
    boolean shooterToggle = false;

    public ShooterControlPrecise(Shooter shooter, Gamepad gamepad) {
        this.shooter = shooter;
        this.gamepad = gamepad;
    }

    public boolean periodic() {
        if (gamepad.getTriangleJustPressed()) {
            shootRpm += 25;
        } else if (gamepad.getCrossJustPressed()) {
            shootRpm -= 25;
        }

        if (gamepad.getDpadUp()) {
            hoodPos += 0.005;
        } else if (gamepad.getDpadDown()) {
            hoodPos -= 0.005;
        }

        shooter.setHood(hoodPos);


        if(gamepad.getSquareJustPressed()){
            shooterToggle = !shooterToggle;
        }

        if (shooterToggle) {
            shooter.spin(shootRpm);
        } else {
            shooter.spin(0);
        }

        if (shootRpm > 0) {
            ShooterControl.percentError = Math.abs(shootRpm - getCurrentRPM()) * 100.0 / shootRpm;
        } else {
            ShooterControl.percentError = -1.0;
        }

        return false;
    }

    public double getShootRpm() {
        return shootRpm;
    }

    public double getCurrentRPM(){
        return shooter.getVelocity();
    }


}
