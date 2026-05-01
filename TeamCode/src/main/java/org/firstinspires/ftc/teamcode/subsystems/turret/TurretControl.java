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
            new Interpolator.Point(new double[]{-27.3, 27.9}, 3150),
            new Interpolator.Point(new double[]{-46.8, 15.3}, 3150),
            new Interpolator.Point(new double[]{-60.3, 10.0}, 3150),
            new Interpolator.Point(new double[]{-60.1, -9.1}, 3600),
            new Interpolator.Point(new double[]{-43.8, -3.9}, 3600),
            new Interpolator.Point(new double[]{-30.3, 4.2}, 3600),
            new Interpolator.Point(new double[]{-12.1, 16.6}, 3600),
            new Interpolator.Point(new double[]{-4.3, 6.1}, 3700),
            new Interpolator.Point(new double[]{-21.6, -5.5}, 3700),
            new Interpolator.Point(new double[]{-46.3, -16.6}, 3800),
            new Interpolator.Point(new double[]{-62.5, -19.6}, 3800),
            new Interpolator.Point(new double[]{-60.7, -31.4}, 4100),
            new Interpolator.Point(new double[]{-47.4, -30.8}, 4100),
            new Interpolator.Point(new double[]{-33.7, -25.1}, 4100),
            new Interpolator.Point(new double[]{2.3, -11.3}, 4150)
    );

    private Interpolator hoodInterpolator = new Interpolator(4,
            new Interpolator.Point(new double[]{-27.3, 27.9}, 0.0),
            new Interpolator.Point(new double[]{-46.8, 15.3}, 0.0),
            new Interpolator.Point(new double[]{-60.3, 10.0}, 0.0),
            new Interpolator.Point(new double[]{-60.1, -9.1}, 0.35),
            new Interpolator.Point(new double[]{-43.8, -3.9}, 0.4),
            new Interpolator.Point(new double[]{-30.3, 4.2}, 0.4),
            new Interpolator.Point(new double[]{-12.1, 16.6}, 0.4),
            new Interpolator.Point(new double[]{-4.3, 6.1}, 0.6),
            new Interpolator.Point(new double[]{-21.6, -5.5}, 0.6),
            new Interpolator.Point(new double[]{-46.3, -16.6}, 0.6),
            new Interpolator.Point(new double[]{-62.5, -19.6}, 0.6),
            new Interpolator.Point(new double[]{-60.7, -31.4}, 0.7),
            new Interpolator.Point(new double[]{-47.4, -30.8}, 0.7),
            new Interpolator.Point(new double[]{-33.7, -25.1}, 0.7),
            new Interpolator.Point(new double[]{2.3, -11.3}, 0.7)
    );

    private Interpolator xTargetInterpolator = new Interpolator(4,
            new Interpolator.Point(new double[]{-27.3, 27.9}, -70.3),
            new Interpolator.Point(new double[]{-46.8, 15.3}, -70.3),
            new Interpolator.Point(new double[]{-60.3, 10.0}, -64.8),
            new Interpolator.Point(new double[]{-60.1, -9.1}, -65.5),
            new Interpolator.Point(new double[]{-43.8, -3.9}, -63.6),
            new Interpolator.Point(new double[]{-30.3, 4.2}, -66.4),
            new Interpolator.Point(new double[]{-12.1, 16.6}, -68.3),
            new Interpolator.Point(new double[]{-4.3, 6.1}, -65.4),
            new Interpolator.Point(new double[]{-21.6, -5.5}, -65.4),
            new Interpolator.Point(new double[]{-46.3, -16.6}, -63.45),
            new Interpolator.Point(new double[]{-62.5, -19.6}, -61.6),
            new Interpolator.Point(new double[]{-60.7, -31.4}, -61.5),
            new Interpolator.Point(new double[]{-47.4, -30.8}, -61.5),
            new Interpolator.Point(new double[]{-33.7, -25.1}, -61.5),
            new Interpolator.Point(new double[]{2.3, -11.3}, -63.0)
    );

    private Interpolator yTargetInterpolator = new Interpolator(4,
            new Interpolator.Point(new double[]{-27.3, 27.9}, 70.3),
            new Interpolator.Point(new double[]{-46.8, 15.3}, 70.3),
            new Interpolator.Point(new double[]{-60.3, 10.0}, 70.3),
            new Interpolator.Point(new double[]{-60.1, -9.1}, 70.3),
            new Interpolator.Point(new double[]{-43.8, -3.9}, 70.3),
            new Interpolator.Point(new double[]{-30.3, 4.2}, 70.3),
            new Interpolator.Point(new double[]{-12.1, 16.6}, 70.3),
            new Interpolator.Point(new double[]{-4.3, 6.1}, 70.3),
            new Interpolator.Point(new double[]{-21.6, -5.5}, 70.3),
            new Interpolator.Point(new double[]{-46.3, -16.6}, 70.3),
            new Interpolator.Point(new double[]{-62.5, -19.6}, 71.5),
            new Interpolator.Point(new double[]{-60.7, -31.4}, 70.3),
            new Interpolator.Point(new double[]{-47.4, -30.8}, 71.5),
            new Interpolator.Point(new double[]{-33.7, -25.1}, 71.5),
            new Interpolator.Point(new double[]{2.3, -11.3}, 70.3)
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