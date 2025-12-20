package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.beaver.command.Command;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterControl implements Command {
    Gamepad gamepad;
    Shooter shooter;

    double shootRpm = 2200.0;
    int shootPos = 1;
    boolean shooterToggle = false;

    public ShooterControl(Shooter shooter, Gamepad gamepad) {
        this.shooter = shooter;
        this.gamepad = gamepad;
    }

    public boolean periodic() {
        if (gamepad.getLeftBumperJustPressed()) {
            shootPos = Math.max(1, shootPos-1);
        } else if (gamepad.getRightBumperJustPressed()) {
            shootPos = Math.min(3, shootPos+1);
        }

        if(shootPos == 1){
            shooter.setHood(0.27);
            shootRpm = 2200.0;
        } else if (shootPos == 2) {
            shooter.setHood(0.35);
            shootRpm = 2500.0;
        }else if (shootPos == 3){
            shooter.setHood(0.53);
            shootRpm = 3000.0;
        }

        if(gamepad.getSquareJustPressed()){
            shooterToggle = !shooterToggle;
        }

        if (shooterToggle) {
            shooter.spin(shootRpm);
        } else {
            shooter.spin(0);
        }

        /*
        if(gamepad.getDpadLeft()){
            shooter.brakesOn();
        }else{
            shooter.brakesOff();
        }
         */

        return false;
    }

    public double getShootRpm() {
        return shootRpm;
    }

    public double getCurrentRPM(){
        return shooter.getVelocity();
    }


}
