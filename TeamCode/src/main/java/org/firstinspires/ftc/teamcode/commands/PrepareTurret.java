package org.firstinspires.ftc.teamcode.commands;

import android.util.Pair;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.TurretV2;
import org.firstinspires.ftc.teamcode.Transform;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.Set;

public class PrepareTurret implements Command {
    private TurretV2 turret;
    private Localizer localizer;
    private Transform goal;

    public PrepareTurret(TurretV2 turret, Localizer localizer, Side side) {
        this.turret = turret;
        this.localizer = localizer;
        if (side == Side.RED) {
            goal = new Transform(Constants.GOAL_X, Constants.GOAL_Y);
        } else {
            goal = new Transform(Constants.GOAL_X, -Constants.GOAL_Y);
        }
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(turret);
    }

    public boolean periodic() {
        double turretAngle = localizer.getPosition().relativeAngleTo(goal);
        turret.setTurretAngle(turretAngle);

        double distance = localizer.getPosition().lateralDistance(goal);

        Pair<Double, Double> settings = turret.getSettings(distance);

        turret.setFlywheelSpeed(settings.first);
        turret.setHoodAngle(settings.second);

        return true;
    }
}
