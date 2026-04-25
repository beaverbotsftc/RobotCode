package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.util.interpolator.Interpolator;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.Set;
import java.util.function.DoubleUnaryOperator;

public class TurretControl implements Command {
    private Turret turret;
    private Localizer localizer;
    private DoubleUnaryOperator[] mirror;

    private Interpolator rpmInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-37.8, 41.3}, 3000),
            new Interpolator.Point(new double[]{-50.4, 15.8}, 3150),
            new Interpolator.Point(new double[]{-30.2, 27.8}, 3150),
            new Interpolator.Point(new double[]{-19.3, 42.2}, 3150),
            new Interpolator.Point(new double[]{-13.2, 24.5}, 3350),
            new Interpolator.Point(new double[]{-26.3, 12.5}, 3350),
            new Interpolator.Point(new double[]{-35.8, 8.2}, 3350),
            new Interpolator.Point(new double[]{-57.0, 1.2}, 3350),
            new Interpolator.Point(new double[]{-64.4, -0.1}, 3350),
            new Interpolator.Point(new double[]{-65.0, -16.1}, 3700),
            new Interpolator.Point(new double[]{-30.3, -8.0}, 3700),
            new Interpolator.Point(new double[]{-9.4, 5.1}, 3700),
            new Interpolator.Point(new double[]{1.8, 12.6}, 3700),
            new Interpolator.Point(new double[]{3.6, -1.7}, 4100),
            new Interpolator.Point(new double[]{-12.3, -16.1}, 4050),
            new Interpolator.Point(new double[]{-29.7, -24.8}, 4100),
            new Interpolator.Point(new double[]{-50.1, -33.5}, 4150),
            new Interpolator.Point(new double[]{-64.4, -38.6}, 4200),
            new Interpolator.Point(new double[]{-49.5, -55.1}, 4550),
            new Interpolator.Point(new double[]{-21.8, -50.8}, 4500),
            new Interpolator.Point(new double[]{-0.9, -37.3}, 4400),
            new Interpolator.Point(new double[]{46.0, -4.8}, 4600)
    );

    private Interpolator hoodInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-37.8, 41.3},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3000}, 0.00)
                    )),
            new Interpolator.Point(new double[]{-50.4, 15.8},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3150}, 0.20)
                    )),
            new Interpolator.Point(new double[]{-30.2, 27.8},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3150}, 0.20)
                    )),
            new Interpolator.Point(new double[]{-19.3, 42.2},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3150}, 0.20)
                    )),
            new Interpolator.Point(new double[]{-13.2, 24.5},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3350}, 0.30)
                    )),
            new Interpolator.Point(new double[]{-26.3, 12.5},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3350}, 0.35)
                    )),
            new Interpolator.Point(new double[]{-35.8, 8.2},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3350}, 0.35)
                    )),
            new Interpolator.Point(new double[]{-57.0, 1.2},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3350}, 0.35)
                    )),
            new Interpolator.Point(new double[]{-64.4, -0.1},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3350}, 0.35)
                    )),
            new Interpolator.Point(new double[]{-65.0, -16.1},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3700}, 0.70)
                    )),
            new Interpolator.Point(new double[]{-30.3, -8.0},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3700}, 0.70)
                    )),
            new Interpolator.Point(new double[]{-9.4, 5.1},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3700}, 0.70)
                    )),
            new Interpolator.Point(new double[]{1.8, 12.6},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3700}, 0.70)
                    )),
            new Interpolator.Point(new double[]{3.6, -1.7},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4100}, 0.85),
                            new Interpolator.Point(new double[]{3900}, 0.70)
                    )
            ),
            new Interpolator.Point(new double[]{-12.3, -16.1},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4050}, 0.85),
                            new Interpolator.Point(new double[]{3850}, 0.75)
                    )
            ),
            new Interpolator.Point(new double[]{-29.7, -24.8},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4100}, 0.80),
                            new Interpolator.Point(new double[]{3900}, 0.75)
                    )
            ),
            new Interpolator.Point(new double[]{-50.1, -33.5},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4150}, 0.80),
                            new Interpolator.Point(new double[]{3950}, 0.65)
                    )
            ),
            new Interpolator.Point(new double[]{-64.4, -38.6},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4200}, 0.85),
                            new Interpolator.Point(new double[]{4000}, 0.65)
                    )
            ),
            new Interpolator.Point(new double[]{-49.5, -55.1},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4550}, 0.90),
                            new Interpolator.Point(new double[]{4350}, 0.70)
                    )
            ),
            new Interpolator.Point(new double[]{-21.8, -50.8},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4500}, 0.85)
                    )),
            new Interpolator.Point(new double[]{-0.9, -37.3},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4400}, 0.90)
                    )),
            new Interpolator.Point(new double[]{46.0, -4.8},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4600}, 1.00),
                            new Interpolator.Point(new double[]{4400}, 0.80)
                    )
            )
    );

    private Interpolator timeInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-37.8, 41.3}, 0.6),
            new Interpolator.Point(new double[]{-50.4, 15.8}, 0.65),
            new Interpolator.Point(new double[]{-30.2, 27.8}, 0.6),
            new Interpolator.Point(new double[]{-13.2, 24.5}, 0.7),
            new Interpolator.Point(new double[]{-26.3, 12.5}, 0.75),
            new Interpolator.Point(new double[]{-35.8, 8.2}, 0.7),
            new Interpolator.Point(new double[]{-57.0, 1.2}, 0.8),
            new Interpolator.Point(new double[]{-64.4, -0.1}, 0.8),
            new Interpolator.Point(new double[]{-65.0, -16.1}, 0.7),
            new Interpolator.Point(new double[]{-30.3, -8.0}, 0.7),
            new Interpolator.Point(new double[]{1.8, 12.6}, 0.7),
            new Interpolator.Point(new double[]{3.6, -1.7}, 0.7),
            new Interpolator.Point(new double[]{-12.3, -16.1}, 0.65),
            new Interpolator.Point(new double[]{-29.7, -24.8}, 0.75),
            new Interpolator.Point(new double[]{-50.1, -33.5}, 0.75),
            new Interpolator.Point(new double[]{-64.4, -38.6}, 0.75),
            new Interpolator.Point(new double[]{-49.5, -55.1}, 0.8),
            new Interpolator.Point(new double[]{-21.8, -50.8}, 0.85),
            new Interpolator.Point(new double[]{-0.9, -37.3}, 0.8),
            new Interpolator.Point(new double[]{46.0, -4.8}, 0.75)
    );

    private Interpolator xTargetInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-37.8, 41.3}, -70.3),
            new Interpolator.Point(new double[]{-50.4, 15.8}, -66.8),
            new Interpolator.Point(new double[]{-30.2, 27.8}, -64.9),
            new Interpolator.Point(new double[]{-19.3, 42.2}, -64.9),
            new Interpolator.Point(new double[]{-13.2, 24.5}, -65.3),
            new Interpolator.Point(new double[]{-26.3, 12.5}, -65.3),
            new Interpolator.Point(new double[]{-35.8, 8.2}, -65.3),
            new Interpolator.Point(new double[]{-57.0, 1.2}, -65.3),
            new Interpolator.Point(new double[]{-64.4, -0.1}, -65.3),
            new Interpolator.Point(new double[]{-65.0, -16.1}, -64.1),
            new Interpolator.Point(new double[]{-30.3, -8.0}, -64.1),
            new Interpolator.Point(new double[]{-9.4, 5.1}, -66.6),
            new Interpolator.Point(new double[]{1.8, 12.6}, -65.2),
            new Interpolator.Point(new double[]{3.6, -1.7}, -65.4),
            new Interpolator.Point(new double[]{-12.3, -16.1}, -64.8),
            new Interpolator.Point(new double[]{-29.7, -24.8}, -66.0),
            new Interpolator.Point(new double[]{-50.1, -33.5}, -65.1),
            new Interpolator.Point(new double[]{-64.4, -38.6}, -64.6),
            new Interpolator.Point(new double[]{-49.5, -55.1}, -64.6),
            new Interpolator.Point(new double[]{-21.8, -50.8}, -64.6),
            new Interpolator.Point(new double[]{-0.9, -37.3}, -67.4),
            new Interpolator.Point(new double[]{46.0, -4.8}, -70.3)
    );

    private Interpolator yTargetInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-37.8, 41.3}, 70.3),
            new Interpolator.Point(new double[]{-50.4, 15.8}, 70.3),
            new Interpolator.Point(new double[]{-30.2, 27.8}, 70.3),
            new Interpolator.Point(new double[]{-19.3, 42.2}, 70.3),
            new Interpolator.Point(new double[]{-13.2, 24.5}, 70.3),
            new Interpolator.Point(new double[]{-26.3, 12.5}, 70.3),
            new Interpolator.Point(new double[]{-35.8, 8.2}, 70.3),
            new Interpolator.Point(new double[]{-57.0, 1.2}, 70.3),
            new Interpolator.Point(new double[]{-64.4, -0.1}, 70.3),
            new Interpolator.Point(new double[]{-65.0, -16.1}, 70.3),
            new Interpolator.Point(new double[]{-30.3, -8.0}, 70.3),
            new Interpolator.Point(new double[]{-9.4, 5.1}, 70.3),
            new Interpolator.Point(new double[]{1.8, 12.6}, 70.3),
            new Interpolator.Point(new double[]{3.6, -1.7}, 70.3),
            new Interpolator.Point(new double[]{-12.3, -16.1}, 70.3),
            new Interpolator.Point(new double[]{-29.7, -24.8}, 70.3),
            new Interpolator.Point(new double[]{-50.1, -33.5}, 70.3),
            new Interpolator.Point(new double[]{-64.4, -38.6}, 70.3),
            new Interpolator.Point(new double[]{-49.5, -55.1}, 70.3),
            new Interpolator.Point(new double[]{-21.8, -50.8}, 70.3),
            new Interpolator.Point(new double[]{-0.9, -37.3}, 70.3),
            new Interpolator.Point(new double[]{46.0, -4.8}, 63.2)
    );

    public TurretControl(Turret turret, Localizer localizer, DoubleUnaryOperator[] mirror) {
        this.turret = turret;
        this.localizer = localizer;
        this.mirror = mirror;
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(turret);
    }

    public boolean periodic() {
        Transform position = localizer.getPosition().transform(mirror);
        Transform velocity = localizer.getVelocity().transform(mirror);
        double time = 0;

        for (int i = 0; i < Constants.turretShootOnTheMoveConvergenceIterations; i++) {
            RobotLog.dd("LOOK HERE LOOK HERE LOOK HERE time", String.valueOf(time));
            RobotLog.dd("LOOK HERE LOOK HERE LOOK HERE pos", position.toString());
            RobotLog.dd("LOOK HERE LOOK HERE LOOK HERE velocity", velocity.toString());
            time = timeInterpolator.evaluate(position.add(velocity.scale(time)).toLateralArray());
        }
        if (time > 2) time = 0;

        Transform effectiveLocation = position.add(velocity.scale(time)).lateral().add(position.angular());

        turret.shoot(rpmInterpolator.evaluate(effectiveLocation.toLateralArray()));
        turret.setHoodAngle(hoodInterpolator.evaluate(effectiveLocation.getX(), effectiveLocation.getY(), turret.getVelocity()));
        //turret.setHoodAngle(hoodInterpolator.evaluate(effectiveLocation.getX(), effectiveLocation.getY(), rpmInterpolator.evaluate(effectiveLocation.toLateralArray())));

        double desiredAngle =
                effectiveLocation.transform(mirror).relativeAngleTo(
                        new Transform(
                                xTargetInterpolator.evaluate(effectiveLocation.toLateralArray()),
                                yTargetInterpolator.evaluate(effectiveLocation.toLateralArray())
                        ).transform(mirror)
                ) - mirror[2].applyAsDouble(velocity.getTheta()) * Constants.turretLatency;

        if (Turret.inBounds(desiredAngle))
            turret.turn(desiredAngle);

        return false;
    }
}