package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverOptimize.IteratedLocalSearch;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterControl implements Command {
    Gamepad gamepad;
    Shooter shooter;

    double shootRpm = 2200.0;
    int shootPos = 1;

    public ShooterControl(Shooter shooter, Gamepad gamepad) {
        this.shooter = shooter;
        this.gamepad = gamepad;
    }

    public boolean periodic() {
        if (gamepad.getTriangle() && shootRpm < 4000) {
            shootRpm += 5;
        }
        if (gamepad.getCross() && shootRpm > 100) {
            shootRpm -= 5;
        }

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
            shootRpm = 2600.0;
        }else if (shootPos == 3){
            shooter.setHood(0.53);
            shootRpm = 3250.0;
        }

        if (gamepad.getSquare()) {
            shooter.spin(shootRpm);

            double velocity = shooter.getVelocity();

            if (Math.abs(velocity - shootRpm) / shootRpm <= 0.016) {
                gamepad.rumble(0.45, 0.45, com.qualcomm.robotcore.hardware.Gamepad.RUMBLE_DURATION_CONTINUOUS);
            } else {
                gamepad.stopRumble();
            }
        } else {
            shooter.spin(0);
        }

        if(gamepad.getDpadLeft()){
            shooter.brakesOn();
        }else{
            shooter.brakesOff();
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
