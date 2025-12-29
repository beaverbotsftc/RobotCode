package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.beaver.command.Command;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterControl implements Command {
    Gamepad gamepad;
    Shooter shooter;
    private Led led;
    double shootRpm = 2200.0;
    int shootPos = 1;
    boolean shooterToggle = false;
    public static double percentError = 0.0;

    public ShooterControl(Shooter shooter, Gamepad gamepad) {
        this.shooter = shooter;
        this.gamepad = gamepad;
    }

    public ShooterControl(Shooter shooter, Led led, Gamepad gamepad) {
        this.shooter = shooter;
        this.led = led;
        this.gamepad = gamepad;
    }

    public boolean periodic() {
        if (gamepad.getLeftBumperJustPressed()) {
            shootPos = Math.max(1, shootPos-1);
        } else if (gamepad.getRightBumperJustPressed()) {
            shootPos = Math.min(3, shootPos+1);
        }

        if(shootPos == 1){
            shooter.setHood(0.30);
            shootRpm = 2200.0;
        } else if (shootPos == 2) {
            shooter.setHood(0.525);
            shootRpm = 2550.0;
        }else if (shootPos == 3){
            shooter.setHood(0.72);
            shootRpm = 3000.0;
        }

        if(gamepad.getSquareJustPressed()){
            shooterToggle = !shooterToggle;
        }

        if (shooterToggle) {
            shooter.spin(shootRpm);
            led.setProximity(shootRpm,getCurrentRPM());
        } else {
            shooter.spin(0);
            led.turnOff();
        }

        if (shootRpm > 0) {
            percentError = Math.abs(shootRpm - getCurrentRPM()) * 100.0 / shootRpm;
        } else {
            percentError = -1.0;
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
