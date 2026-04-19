package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.util.interpolator.Interpolator;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.Set;

public class TurretControl implements Command {
    private Turret turret;
    private Localizer localizer;

    private Interpolator rpmInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-38, 39}, 3000),
            new Interpolator.Point(new double[]{-18.4, 23.2}, 3350),
            new Interpolator.Point(new double[]{-44.1, 14.2}, 3100),
            new Interpolator.Point(new double[]{-63.3, 4.6}, 3250),
            new Interpolator.Point(new double[]{-63.3, 22.9}, 2900),
            new Interpolator.Point(new double[]{8.2, 14.0}, 3850),
            new Interpolator.Point(new double[]{-2.7, 41.0}, 3150),
            new Interpolator.Point(new double[]{-25.8, 38.6}, 3000),
            new Interpolator.Point(new double[]{-1.7, -1.6}, 3900)
    );

    private Interpolator hoodInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-38, 39},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3600}, 0)
                    )
            ),
            new Interpolator.Point(new double[]{-18.4, 23.2},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3350}, 0.4),
                            new Interpolator.Point(new double[]{3200}, 0.55)
                    )
            ),
            new Interpolator.Point(new double[]{-44.1, 14.2},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3100}, 0.45),
                            new Interpolator.Point(new double[]{2950}, 0.35)
                    )
            ),
            new Interpolator.Point(new double[]{-63.3, 4.6},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3250}, 0.4),
                            new Interpolator.Point(new double[]{3050}, 0.35)
                    )
            ),
            new Interpolator.Point(new double[]{-63.3, 22.9},
                    new Interpolator(
                            new Interpolator.Point(new double[]{2900}, 0)
                    )
            ),
            new Interpolator.Point(new double[]{8.2, 14.0},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3850}, 0.9),
                            new Interpolator.Point(new double[]{3600}, 0.7)
                    )
            ),
            new Interpolator.Point(new double[]{-2.7, 41.0},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3150}, 0.35),
                            new Interpolator.Point(new double[]{2950}, 0.1)
                    )
            ),
            new Interpolator.Point(new double[]{-25.8, 38.6},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3000}, 0)
                    )
            ),
            new Interpolator.Point(new double[]{-1.7, -1.6},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3900}, 0.7),
                            new Interpolator.Point(new double[]{3750}, 0.6)
                    )
            )
    );


    private Interpolator timeInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-38, 39}, 0.71),
            new Interpolator.Point(new double[]{-18.4, 23.2}, 0.85),
            new Interpolator.Point(new double[]{-44.1, 14.2}, 0.7),
            new Interpolator.Point(new double[]{-63.3, 4.6}, 0.7),
            new Interpolator.Point(new double[]{-63.3, 22.9}, 0.6),
            new Interpolator.Point(new double[]{8.2, 14.0}, 0.9),
            new Interpolator.Point(new double[]{-2.7, 41.0}, 0.7),
            new Interpolator.Point(new double[]{-25.8, 38.6}, 0.6),
            new Interpolator.Point(new double[]{-1.7, -1.6}, 0.9)
    );

    private Interpolator xInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-38, 39}, -70.3),
            new Interpolator.Point(new double[]{-18.4, 23.2}, -62.8),
            new Interpolator.Point(new double[]{-44.1, 14.2}, -61),
            new Interpolator.Point(new double[]{-63.3, 4.6}, -64.4),
            new Interpolator.Point(new double[]{-63.3, 22.9}, -64.4),
            new Interpolator.Point(new double[]{8.2, 14.0}, -62.0),
            new Interpolator.Point(new double[]{-2.7, 41.0}, -64.6),
            new Interpolator.Point(new double[]{-25.8, 38.6}, -64.6),
            new Interpolator.Point(new double[]{-1.7, -1.6}, -62.8)
    );
    private Interpolator yInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-38, 39}, 70.3),
            new Interpolator.Point(new double[]{-18.4, 23.2}, 70.3),
            new Interpolator.Point(new double[]{-44.1, 14.2}, 70.3),
            new Interpolator.Point(new double[]{-63.3, 4.6}, 70.3),
            new Interpolator.Point(new double[]{-63.3, 22.9}, 70.3),
            new Interpolator.Point(new double[]{8.2, 14.0}, 70.3),
            new Interpolator.Point(new double[]{-2.7, 41.0}, 70.3),
            new Interpolator.Point(new double[]{-25.8, 38.6}, 70.3),
            new Interpolator.Point(new double[]{-1.7, -1.6}, 70.3)
    );

    public TurretControl(Turret turret, Localizer localizer) {
        this.turret = turret;
        this.localizer = localizer;
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(turret);
    }

    public boolean periodic() {
        Transform position = localizer.getPosition();
        Transform velocity = localizer.getVelocity();
        double time = 0;

        for (int i = 0; i < Constants.turretShootOnTheMoveConvergenceIterations; i++)
            time = timeInterpolator.evaluate(position.add(velocity.scale(time)).toArray());
        if (time > 2) time = 0;

        Transform effectiveLocation = position.add(velocity.scale(time)).lateral().add(position.angular());

        turret.shoot(rpmInterpolator.evaluate(effectiveLocation.getX(), effectiveLocation.getY()));
        turret.setHoodAngle(hoodInterpolator.evaluate(effectiveLocation.getX(), effectiveLocation.getY(), turret.getVelocity()));
        turret.turn(
                effectiveLocation.relativeAngleTo(
                        new Transform(
                                xInterpolator.evaluate(effectiveLocation.getX(), effectiveLocation.getY()),
                                yInterpolator.evaluate(effectiveLocation.getX(), effectiveLocation.getY())
                        )
                ) - velocity.getTheta() * Constants.turretLatency
        );

        return false;
    }
}
