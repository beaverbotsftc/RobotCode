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
            new Interpolator.Point(new double[]{-43.7, 19.8}, 3200),
            new Interpolator.Point(new double[]{-48.2, 14.5}, 3200),
            new Interpolator.Point(new double[]{-61.7, 12.1}, 3200),
            new Interpolator.Point(new double[]{-62.8, -3.5}, 3300),
            new Interpolator.Point(new double[]{-49.8, 0.8}, 3300),
            new Interpolator.Point(new double[]{-35.8, 3.7}, 3300),
            new Interpolator.Point(new double[]{-20.0, 12.7}, 3300),
            new Interpolator.Point(new double[]{-10.6, 3.4}, 3500),
            new Interpolator.Point(new double[]{-21.4, -5.9}, 3500),
            new Interpolator.Point(new double[]{-36.1, -14.1}, 3500),
            new Interpolator.Point(new double[]{-50.2, -16.2}, 3500),
            new Interpolator.Point(new double[]{-61.4, -18.1}, 3500),
            new Interpolator.Point(new double[]{-63.2, -28.8}, 3800),
            new Interpolator.Point(new double[]{-44.5, -26.5}, 3800),
            new Interpolator.Point(new double[]{-27.7, -22.0}, 3800),
            new Interpolator.Point(new double[]{-11.0, -10.7}, 3800),
            new Interpolator.Point(new double[]{2.5, 1.2}, 3800),
            //new Interpolator.Point(new double[]{64.0, 22.7}, 4600)
            //new Interpolator.Point(new double[]{60.3, 13.1}, 4750)
            //new Interpolator.Point(new double[]{64.8, 22.7}, 4550)
            //new Interpolator.Point(new double[]{62, 16}, 4550)
            new Interpolator.Point(new double[]{61.6, 8.1}, 4800),
            new Interpolator.Point(new double[]{64.3, -0.3}, 4800),
            new Interpolator.Point(new double[]{63.5, -7.0}, 4950),
            new Interpolator.Point(new double[]{63.5, -7.0}, 5100),
            new Interpolator.Point(new double[]{53.9, -17.6}, 4850),
            new Interpolator.Point(new double[]{53.2, -10.8}, 4950),
            new Interpolator.Point(new double[]{48.4, -1.0}, 4700),
            new Interpolator.Point(new double[]{51.8, 5.7}, 4600)
    );

    private Interpolator hoodInterpolator = new Interpolator(4,
            new Interpolator.Point(new double[]{-43.7, 19.8}, 0.0),
            new Interpolator.Point(new double[]{-48.2, 14.5}, 0.0),
            new Interpolator.Point(new double[]{-61.7, 12.1}, 0.0),
            new Interpolator.Point(new double[]{-62.8, -3.5}, 0.0),
            new Interpolator.Point(new double[]{-49.8, 0.8}, 0.0),
            new Interpolator.Point(new double[]{-35.8, 3.7}, 0.0),
            new Interpolator.Point(new double[]{-20.0, 12.7}, 0.0),
            new Interpolator.Point(new double[]{-10.6, 3.4}, 0.2),
            new Interpolator.Point(new double[]{-21.4, -5.9}, 0.2),
            new Interpolator.Point(new double[]{-36.1, -14.1}, 0.2),
            new Interpolator.Point(new double[]{-50.2, -16.2}, 0.2),
            new Interpolator.Point(new double[]{-61.4, -18.1}, 0.2),
            new Interpolator.Point(new double[]{-63.2, -28.8}, 0.3),
            new Interpolator.Point(new double[]{-44.5, -26.5}, 0.3),
            new Interpolator.Point(new double[]{-27.7, -22.0}, 0.3),
            new Interpolator.Point(new double[]{-11.0, -10.7}, 0.3),
            new Interpolator.Point(new double[]{2.5, 1.2}, 0.3),
            //new Interpolator.Point(new double[]{64.0, 22.7}, 0.6),
            //new Interpolator.Point(new double[]{60.3, 13.1}, 0.8)
            //new Interpolator.Point(new double[]{64.8, 22.7}, 0.7)
            //new Interpolator.Point(new double[]{62, 16}, 0.7)
            new Interpolator.Point(new double[]{61.6, 8.1}, 0.75),
            new Interpolator.Point(new double[]{64.3, -0.3}, 0.75),
            new Interpolator.Point(new double[]{63.5, -7.0}, 0.8),
            new Interpolator.Point(new double[]{61.8, -4.6}, 0.8),
            new Interpolator.Point(new double[]{63.5, -7.0}, 0.8),
            new Interpolator.Point(new double[]{53.9, -17.6}, 0.75),
            new Interpolator.Point(new double[]{53.2, -10.8}, 0.8),
            new Interpolator.Point(new double[]{48.4, -1.0}, 0.8),
            new Interpolator.Point(new double[]{51.8, 5.7}, 0.8)
    );

    private Interpolator xTargetInterpolator = new Interpolator(4,
            new Interpolator.Point(new double[]{-43.7, 19.8}, -70.3),
            new Interpolator.Point(new double[]{-48.2, 14.5}, -65.4),
            new Interpolator.Point(new double[]{-61.7, 12.1}, -65.4),
            new Interpolator.Point(new double[]{-62.8, -3.5}, -65.4),
            new Interpolator.Point(new double[]{-49.8, 0.8}, -65.4),
            new Interpolator.Point(new double[]{-35.8, 3.7}, -65.5),
            new Interpolator.Point(new double[]{-20.0, 12.7}, -67.7),
            new Interpolator.Point(new double[]{-10.6, 3.4}, -67.7),
            new Interpolator.Point(new double[]{-21.4, -5.9}, -66.8),
            new Interpolator.Point(new double[]{-36.1, -14.1}, -61.9 - 5),
            new Interpolator.Point(new double[]{-50.2, -16.2}, -61.9 - 5),
            new Interpolator.Point(new double[]{-61.4, -18.1}, -61.9 - 5),
            new Interpolator.Point(new double[]{-63.2, -28.8}, -61.9 - 5),
            new Interpolator.Point(new double[]{-44.5, -26.5}, -61.9 - 5),
            new Interpolator.Point(new double[]{-27.7, -22.0}, -61.9 - 5),
            new Interpolator.Point(new double[]{-11.0, -10.7}, -65.4),
            new Interpolator.Point(new double[]{2.5, 1.2}, -66.3),
            //new Interpolator.Point(new double[]{64.0, 22.7}, -70., 0.)
            //new Interpolator - 5.Point(new double[]{60.3, 13.1}, -70., 0.013)
            //new Interpolator.Point(new double[]{64.8, 22.7}, -70., 0.013)
            //new Interpolator - 5.Point(new double[]{62, 16}, -70.3, 0.01)
            new Interpolator.Point(new double[]{61.6, 8.1}, -70.3),
            new Interpolator.Point(new double[]{64.3, -0.3}, -70.3),
            new Interpolator.Point(new double[]{63.5, -7.0}, -70.3),
            new Interpolator.Point(new double[]{61.8, -4.6}, -70.3),
            new Interpolator.Point(new double[]{63.5, -7.0}, -70.3),
            new Interpolator.Point(new double[]{53.9, -17.6}, -70.3),
            new Interpolator.Point(new double[]{53.2, -10.8}, -70.3),
            new Interpolator.Point(new double[]{48.4, -1.0}, -70.3),
            new Interpolator.Point(new double[]{51.8, 5.7}, -70.3)
    );

    private Interpolator yTargetInterpolator = new Interpolator(4,
            new Interpolator.Point(new double[]{-43.7, 19.8}, 70.3),
            new Interpolator.Point(new double[]{-48.2, 14.5}, 70.3),
            new Interpolator.Point(new double[]{-61.7, 12.1}, 70.3),
            new Interpolator.Point(new double[]{-62.8, -3.5}, 70.3),
            new Interpolator.Point(new double[]{-49.8, 0.8}, 70.3),
            new Interpolator.Point(new double[]{-35.8, 3.7}, 70.3),
            new Interpolator.Point(new double[]{-20.0, 12.7}, 70.3),
            new Interpolator.Point(new double[]{-10.6, 3.4}, 70.3),
            new Interpolator.Point(new double[]{-21.4, -5.9}, 70.3),
            new Interpolator.Point(new double[]{-36.1, -14.1}, 70.3),
            new Interpolator.Point(new double[]{-50.2, -16.2}, 70.3),
            new Interpolator.Point(new double[]{-61.4, -18.1}, 70.3),
            new Interpolator.Point(new double[]{-63.2, -28.8}, 70.3),
            new Interpolator.Point(new double[]{-44.5, -26.5}, 70.3),
            new Interpolator.Point(new double[]{-27.7, -22.0}, 70.3),
            new Interpolator.Point(new double[]{-11.0, -10.7}, 70.3),
            new Interpolator.Point(new double[]{2.5, 1.2}, 70.3),
            //new Interpolator.Point(new double[]{64.0, 22.7}, 67.1)
            //new Interpolator.Point(new double[]{60.3, 13.1}, 61.0)
            //new Interpolator.Point(new double[]{64.8, 22.7}, 67.7)
            //new Interpolator.Point(new double[]{62, 16}, 67.7)
            new Interpolator.Point(new double[]{61.6, 8.1}, 66.0),
            new Interpolator.Point(new double[]{64.3, -0.3}, 66.0),
            new Interpolator.Point(new double[]{63.5, -7.0}, 66.0),
            new Interpolator.Point(new double[]{61.8, -4.6}, 67.5),
            new Interpolator.Point(new double[]{63.5, -7.0}, 70.3),
            new Interpolator.Point(new double[]{53.9, -17.6}, 70.3),
            new Interpolator.Point(new double[]{53.2, -10.8}, 67.4),
            new Interpolator.Point(new double[]{48.4, -1.0}, 67.4),
            new Interpolator.Point(new double[]{51.8, 5.7}, 65.9)
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
            //new Interpolator.Point(new double[]{46.0, -4.8}, 0.75),
            //new Interpolator.Point(new double[]{62.5, 16.3}, 0.9),
            //new Interpolator.Point(new double[] {61.4, -12.9}, 0.95)
            new Interpolator.Point(new double[]{61.6, 8.1}, 0.85),
            new Interpolator.Point(new double[]{64.3, -0.3}, 0.8),
            new Interpolator.Point(new double[]{63.5, -7.0}, 0.8),
            new Interpolator.Point(new double[]{61.8, -4.6}, 0.8),
            new Interpolator.Point(new double[]{63.5, -7.0}, 0.9),
            new Interpolator.Point(new double[]{53.9, -17.6}, 0.75),
            new Interpolator.Point(new double[]{53.2, -10.8}, 0.8),
            new Interpolator.Point(new double[]{48.4, -1.0}, 0.75),
            new Interpolator.Point(new double[]{51.8, 5.7}, 0.8)
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