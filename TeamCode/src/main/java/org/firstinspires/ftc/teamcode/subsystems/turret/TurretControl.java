package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.util.Interpolator;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.Set;

public class TurretControl implements Command {
    private Turret turret;
    private Localizer localizer;
    private Transform target;

    private Interpolator rpmInterpolator = new Interpolator(
            new Interpolator.Point(new double[] {0, 0}, 3600),
            new Interpolator.Point(new double[] {-36, 36}, 3200),
            new Interpolator.Point(new double[] {-72, 0}, 3200)
    );

    private Interpolator hoodInterpolator = new Interpolator(
            new Interpolator.Point(new double[] {0, 0, 3600}, 0),
            new Interpolator.Point(new double[] {-36, 36, 3200}, 0),
            new Interpolator.Point(new double[] {-72, 0, 3200}, 0)
    );

    private Interpolator timeInterpolator = new Interpolator(
            new Interpolator.Point(new double[] {0, 0}, 1),
            new Interpolator.Point(new double[] {-36, 36}, 0.8),
            new Interpolator.Point(new double[] {-72, 0}, 0.8)
    );

    public TurretControl(Turret turret, Localizer localizer, Transform target) {
        this.turret = turret;
        this.localizer = localizer;
        this.target = target;
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(turret);
    }

    public boolean periodic() {
        double angle = localizer.getPosition().add(new Transform(0, 0, localizer.getVelocity().getTheta() * Constants.turretLatency)).relativeAngleTo(target);
        if (Turret.inBounds(angle))
            turret.turn(angle);

        return false;
    }
}
