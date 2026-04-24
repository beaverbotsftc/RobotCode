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
            new Interpolator.Point(new double[]{-30.2, 27.8}, 3150)
    );

    private Interpolator hoodInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-37.8, 41.3},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3000}, 0)
                    )),
            new Interpolator.Point(new double[]{-50.4, 15.8},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3150}, 0.2)
                    )
            ),
            new Interpolator.Point(new double[]{-30.2, 27.8},
                    new Interpolator(
                            new Interpolator.Point(new double[]{3150}, 0.2)
                    )
            )
    );


    private Interpolator timeInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-37.8, 41.3}, 0.9 - 0.45),
            new Interpolator.Point(new double[]{-50.4, 15.8}, 1.18-0.56),
            new Interpolator.Point(new double[]{-30.2, 27.8}, 1.72-1.2)
    );

    private Interpolator xInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-37.8, 41.3}, -70.3),
            new Interpolator.Point(new double[]{-50.4, 15.8}, -66.8),
            new Interpolator.Point(new double[]{-30.2, 27.8}, -64.9)
    );
    private Interpolator yInterpolator = new Interpolator(
            new Interpolator.Point(new double[]{-37.8, 41.3}, 70.3),
            new Interpolator.Point(new double[]{-50.4, 15.8}, 70.3),
            new Interpolator.Point(new double[]{-30.2, 27.8}, 70.3)
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
                                xInterpolator.evaluate(effectiveLocation.toLateralArray()),
                                yInterpolator.evaluate(effectiveLocation.toLateralArray())
                        ).transform(mirror)
                ) - mirror[2].applyAsDouble(velocity.getTheta()) * Constants.turretLatency;

        if (Turret.inBounds(desiredAngle))
            turret.turn(desiredAngle);

        return false;
    }
}