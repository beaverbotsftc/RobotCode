package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.Set;

public class TrackTarget implements Command {
    private Turret turret;
    private Localizer localizer;
    private Transform target;

    public TrackTarget(Turret turret, Localizer localizer, Transform target) {
        this.turret = turret;
        this.localizer = localizer;
        this.target = target;
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(turret);
    }

    public boolean periodic() {
        double angle = localizer.getPosition().add(localizer.getVelocity().scale(Constants.turretLatency)).relativeAngleTo(target);
        if (Turret.inBounds(angle))
            turret.turn(angle);
        return false;
    }
}
