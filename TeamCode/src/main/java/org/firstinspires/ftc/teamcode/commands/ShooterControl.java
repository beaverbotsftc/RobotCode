package org.firstinspires.ftc.teamcode.commands;

import android.util.Pair;

import org.beaverbots.beaver.command.Command;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

public class ShooterControl implements Command {
    private Gamepad gamepad;
    private Shooter shooter;
    private Localizer localizer;
    private Led led;
    private Side side;
    double shootRpm = 2200.0;
    boolean shooterToggle = true;
    private boolean adjustRpm;
    public static double percentError = 0.0;

    public ShooterControl(Shooter shooter, Localizer localizer, Side side, Gamepad gamepad) {
        this.shooter = shooter;
        this.localizer = localizer;
        this.gamepad = gamepad;
        this.side = side;
    }

    public ShooterControl(Shooter shooter, Localizer localizer, boolean adjustRpm, Side side, Led led, Gamepad gamepad) {
        this.shooter = shooter;
        this.localizer = localizer;
        this.led = led;
        this.gamepad = gamepad;
        this.side = side;
        this.adjustRpm = adjustRpm;
    }

    public boolean periodic() {
        double distToTarget = localizer.getPosition().lateralDistance(new DrivetrainState(Constants.GOAL_X, side == Side.RED ? Constants.GOAL_Y : -Constants.GOAL_Y, 0));
/*
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
            shootRpm = 2950.0;
        }
 */
        Pair<Double, Double> values = shooter.getSettingsAtDistance(distToTarget);
        shootRpm = values.first;

        shooter.setHood(values.second);


        if(gamepad.getLeftBumperJustPressed()){
            shooterToggle = !shooterToggle;
        }

        if (shooterToggle) {
            if (adjustRpm)
                shooter.spin(shootRpm);
            led.setProximity(shootRpm,getCurrentRPM());
        } else {
            shooter.spin(0);
            led.turnOffRPMLed();
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

    public double getDistanceToTag(){
        return localizer.getPosition().lateralDistance(new DrivetrainState(Constants.GOAL_X, side == Side.RED ? Constants.GOAL_Y : -Constants.GOAL_Y, 0));
    }
}
