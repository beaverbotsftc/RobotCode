package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.util.Geometry;
import org.beaverbots.beaver.util.Pair;
import org.beaverbots.beaver.util.interpolator.Interpolator;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;
import org.firstinspires.ftc.teamcode.teleop.TheTeleOpOfTheRobot;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleUnaryOperator;

public class TurretControl implements Command {
    private Turret turret;
    private Localizer localizer;
    private DoubleUnaryOperator[] mirror;

    private Interpolator rpmInterpolator = new Interpolator(4,
            new Interpolator.Point(new double[]{-37.8, 41.3}, 2950),
            new Interpolator.Point(new double[]{-50.4, 15.8}, 3100),
            new Interpolator.Point(new double[]{-30.2, 27.8}, 3150),
            new Interpolator.Point(new double[]{-19.3, 42.2}, 3150),
            new Interpolator.Point(new double[]{-13.2, 24.5}, 3300),
            new Interpolator.Point(new double[]{-26.3, 12.5}, 3350),
            new Interpolator.Point(new double[]{-35.8, 8.2}, 3350),
            new Interpolator.Point(new double[]{-57.0, 1.2}, 3350),
            new Interpolator.Point(new double[]{-64.4, -0.1}, 3350),
            new Interpolator.Point(new double[]{-65.0, -16.1}, 3600),
            new Interpolator.Point(new double[]{-30.3, -8.0}, 3600),
            new Interpolator.Point(new double[]{-9.4, 5.1}, 3600),
            new Interpolator.Point(new double[]{1.8, 12.6}, 3550),
            new Interpolator.Point(new double[]{3.6, -1.7}, 4000),
            new Interpolator.Point(new double[]{-12.3, -16.1}, 4000),
            new Interpolator.Point(new double[]{-29.7, -24.8}, 3950),
            new Interpolator.Point(new double[]{-50.1, -33.5}, 4000),
            new Interpolator.Point(new double[]{-64.4, -38.6}, 4100),
            new Interpolator.Point(new double[]{-49.5, -55.1}, 4450),
            new Interpolator.Point(new double[]{-21.8, -50.8}, 4400),
            new Interpolator.Point(new double[]{-0.9, -37.3}, 4400 - 150),
            new Interpolator.Point(new double[]{46.0, -4.8}, 4700),
            new Interpolator.Point(new double[]{62.5, 16.3}, 4550),
            new Interpolator.Point(new double[] {61.4, -12.9}, 4900)
    );

    private Interpolator hoodInterpolator = new Interpolator(4,
            new Interpolator.Point(new double[]{-37.8, 41.3},
                    new Interpolator(
                            new Interpolator.Point(new double[]{2950}, 0.00)
                    )),
            new Interpolator.Point(new double[]{-50.4, 15.8},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3100}, 0.40)
                    )),
            new Interpolator.Point(new double[]{-30.2, 27.8},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3150}, 0.40 + 0.1 - 0.3333)
                    )),
            new Interpolator.Point(new double[]{-19.3, 42.2},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3150}, 0.40 + 0.1 - 0.3333)
                    )),
            new Interpolator.Point(new double[]{-13.2, 24.5},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3300}, 0.40 + 0.1 - 0.3333)
                    )),
            new Interpolator.Point(new double[]{-26.3, 12.5},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3350}, 0.40 + 0.1 - 0.3333)
                    )),
            new Interpolator.Point(new double[]{-35.8, 8.2},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3350}, 0.40 + 0.1 - 0.3333)
                    )),
            new Interpolator.Point(new double[]{-57.0, 1.2},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3350}, 0.40 + 0.1 - 0.3333)
                    )),
            new Interpolator.Point(new double[]{-64.4, -0.1},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3350}, 0.40 + 0.1 - 0.3333)
                    )),
            new Interpolator.Point(new double[]{-65.0, -16.1},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3700}, 0.65 + 0.1 - 0.3333)
                    )),
            new Interpolator.Point(new double[]{-30.3, -8.0},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3700}, 0.65 + 0.1 - 0.3333)
                    )),
            new Interpolator.Point(new double[]{-9.4, 5.1},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3700}, 0.65 + 0.1 - 0.3333)
                    )),
            new Interpolator.Point(new double[]{1.8, 12.6},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3700}, 0.65 + 0.1 - 0.3333)
                    )),
            new Interpolator.Point(new double[]{3.6, -1.7},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4000}, 1.00),
                            new Interpolator.Point(new double[]{3750}, 0.95)
                    )
            ),
            new Interpolator.Point(new double[]{-12.3, -16.1},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4050}, 1.00),
                            new Interpolator.Point(new double[]{3850}, 0.75)
                    )
            ),
            new Interpolator.Point(new double[]{-29.7, -24.8},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3950}, 0.95),
                            new Interpolator.Point(new double[]{3750}, 0.95) // Interesting...
                    )
            ),
            new Interpolator.Point(new double[]{-50.1, -33.5},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4000}, 1.00),
                            new Interpolator.Point(new double[]{3800}, 0.90)
                    )
            ),
            new Interpolator.Point(new double[]{-64.4, -38.6},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4100}, 1.00),
                            new Interpolator.Point(new double[]{3900}, 1.00) // Interesting
                    )
            ),
            new Interpolator.Point(new double[]{-49.5, -55.1},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4450}, 0.90),
                            new Interpolator.Point(new double[]{4250}, 0.95)
                    )
            ),
            new Interpolator.Point(new double[]{-21.8, -50.8},
                    new Interpolator(
                            new Interpolator.Point(new double[]{4400}, 0.9),
                            new Interpolator.Point(new double[]{4200}, 0.85)
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
            ),
            new Interpolator.Point(new double[]{62.5, 16.3},
                    new Interpolator(
                            new Interpolator.Point(
                                    new double[]{4550}, 1.00
                            ),
                            new Interpolator.Point(
                                    new double[]{4350}, 0.90
                            )
                    )
            ),
            new Interpolator.Point(new double[] {61.4, -12.9}, new Interpolator(
                    new Interpolator.Point(new double[] {4900}, 1.00),
                    new Interpolator.Point(new double[] {4700}, 0.90)
            ))
    );

    private Interpolator timeInterpolator = new Interpolator(4,
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
            new Interpolator.Point(new double[]{46.0, -4.8}, 0.75),
            new Interpolator.Point(new double[]{62.5, 16.3}, 0.9),
            new Interpolator.Point(new double[] {61.4, -12.9}, 0.95)
    );

    private Interpolator xTargetInterpolator = new Interpolator(4,
            new Interpolator.Point(new double[]{-37.8, 41.3}, -70.3),
            new Interpolator.Point(new double[]{-50.4, 15.8}, -66.0),
            new Interpolator.Point(new double[]{-30.2, 27.8}, -70.3),
            new Interpolator.Point(new double[]{-19.3, 42.2}, -70.3),
            new Interpolator.Point(new double[]{-13.2, 24.5}, -70.3),
            new Interpolator.Point(new double[]{-26.3, 12.5}, -66.0),
            new Interpolator.Point(new double[]{-35.8, 8.2}, -66.0),
            new Interpolator.Point(new double[]{-57.0, 1.2}, -66.0),
            new Interpolator.Point(new double[]{-64.4, -0.1}, -63.7),
            new Interpolator.Point(new double[]{-65.0, -16.1}, -63.7),
            new Interpolator.Point(new double[]{-30.3, -8.0}, -66.0),
            new Interpolator.Point(new double[]{-9.4, 5.1}, -66.0),
            new Interpolator.Point(new double[]{1.8, 12.6}, -64.6),
            new Interpolator.Point(new double[]{3.6, -1.7}, -68.2),
            new Interpolator.Point(new double[]{-12.3, -16.1}, -66.2),
            new Interpolator.Point(new double[]{-29.7, -24.8}, -66.5),
            new Interpolator.Point(new double[]{-50.1, -33.5}, -66.5),
            new Interpolator.Point(new double[]{-64.4, -38.6}, -64.2),
            new Interpolator.Point(new double[]{-49.5, -55.1}, -64.6),
            new Interpolator.Point(new double[]{-21.8, -50.8}, -67.8),
            new Interpolator.Point(new double[]{-0.9, -37.3}, -67.4),
            new Interpolator.Point(new double[]{46.0, -4.8}, -70.3),
            new Interpolator.Point(new double[]{62.5, 16.3}, -70.3),
            new Interpolator.Point(new double[] {61.4, -12.9}, -70.3)
    );

    private Interpolator yTargetInterpolator = new Interpolator(4,
            new Interpolator.Point(new double[]{-37.8, 41.3}, 70.3),
            new Interpolator.Point(new double[]{-50.4, 15.8}, 70.3),
            new Interpolator.Point(new double[]{-30.2, 27.8}, 70.3),
            new Interpolator.Point(new double[]{-19.3, 42.2}, 70.3),
            new Interpolator.Point(new double[]{-13.2, 24.5}, 66.5),
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
            new Interpolator.Point(new double[]{46.0, -4.8}, 63.2),
            //new Interpolator.Point(new double[]{62.5, 16.3}, 63.3),
            new Interpolator.Point(new double[]{62.5, 16.3}, 67.0),
            new Interpolator.Point(new double[]{61.4, -12.9}, 63.4)
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
        final List<Double> launchZoneX = List.of(0.0, -72.0, -72.0);
        final List<Double> launchZoneY = List.of(0.0, -72.0, 72.0);
        final List<Double> farLaunchZoneX = List.of(48.0, 72.0, 72.0);
        final List<Double> farLaunchZoneY = List.of(0.0, 24.0, -24.0);

        Pair<List<Double>, List<Double>> robot = Geometry.generateBox(localizer.getPosition().getX(), localizer.getPosition().getY(), 25, 25, localizer.getPosition().getTheta());
        //if (!Geometry.polygonPolygonIntersects(launchZoneX, launchZoneY, robot.first, robot.second) && !Geometry.polygonPolygonIntersects(farLaunchZoneX, farLaunchZoneY, robot.first, robot.second))
        //    return false;

        Transform position = localizer.getPosition().transform(mirror);
        Transform velocity = localizer.getVelocity().transform(mirror);
        double time = 0;

        for (int i = 0; i < Constants.turretShootOnTheMoveConvergenceIterations; i++) {
            try {
                time = timeInterpolator.evaluate(position.add(velocity.scale(time)).toLateralArray());
            } catch (Exception e) {
                RobotLog.dd("LOOK HERE LOOK HERE LOOK HERE time", String.valueOf(time));
                RobotLog.dd("LOOK HERE LOOK HERE LOOK HERE pos", position.toString());
                RobotLog.dd("LOOK HERE LOOK HERE LOOK HERE velocity", velocity.toString());
                return false;
            }
        }
        if (time > 2) time = 0;

        Transform effectiveLocation = position.add(velocity.scale(time)).lateral().add(position.angular());

        turret.shoot(rpmInterpolator.evaluate(effectiveLocation.toLateralArray()) + TheTeleOpOfTheRobot.offset);
        turret.setHoodAngle(hoodInterpolator.evaluate(effectiveLocation.getX(), effectiveLocation.getY(), turret.getVelocity()));
        //turret.setHoodAngle(hoodInterpolator.evaluate(effectiveLocation.getX(), effectiveLocation.getY(), rpmInterpolator.evaluate(effectiveLocation.toLateralArray())));

        double desiredAngle =
                effectiveLocation.transform(mirror).relativeAngleTo(
                        new Transform(
                                xTargetInterpolator.evaluate(effectiveLocation.toLateralArray()),
                                yTargetInterpolator.evaluate(effectiveLocation.toLateralArray())
                        ).transform(mirror)
                ) - mirror[2].applyAsDouble(velocity.getTheta()) * Constants.turretLatency;

        //if (Turret.inBounds(desiredAngle))
            turret.turn(desiredAngle);

        return false;
    }
}