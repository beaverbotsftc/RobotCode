package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.PathAxis;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.pathing.trackers.HolonomicPathTracker;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Transform;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.List;
import java.util.Set;

public class AimWhileDriving implements Command {
    private Localizer localizer;
    private Drivetrain drivetrain;
    private Gamepad gamepad;
    private boolean wall;

    private Stopwatch stopwatch;

    private Side side;
    private HolonomicPathTracker aimTracker;

    public AimWhileDriving(Localizer localization, Drivetrain drivetrain, Side side, Gamepad gamepad, boolean wall) {
        this.localizer = localization;
        this.drivetrain = drivetrain;
        this.side = side;
        this.gamepad = gamepad;
        this.wall = wall;
        this.stopwatch = new Stopwatch();
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(drivetrain);
    }

    public void start() {
        Transform goalPosition = new Transform(Constants.GOAL_X, side == Side.RED ? Constants.GOAL_Y : -Constants.GOAL_Y, 0);

        aimTracker = new HolonomicPathTracker(
                new Path(
                        List.of(
                                new PathAxis(t ->
                                        Localizer.wind(
                                                wall
                                                        ? (side == Side.RED ? Math.PI / 2 : -Math.PI / 2)
                                                        : (localizer.getPosition().angleTo(goalPosition) - Constants.shooterBias),
                                               localizer.getPosition().getTheta()
                                        ), 0, Double.POSITIVE_INFINITY)
                        ),
                        t -> false
                ),
                new PIDF(List.of(
                        new PIDFAxis(new PIDFAxis.K(2 * Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 0, 6, 48, Constants.pidTauTheta, Constants.pidGammaTheta))
                ))
        );

        stopwatch.reset();
    }

    public boolean periodic() {
        double headingCorrection = aimTracker.update(List.of(localizer.getPosition().getTheta()), stopwatch.getDt()).get(0);
        double reverse = side == Side.RED ? 1 : -1;
        drivetrain.move(new Transform(reverse * gamepad.getLeftX() / Constants.drivetrainPowerConversionFactorX, reverse * gamepad.getLeftY() / Constants.drivetrainPowerConversionFactorY, headingCorrection), localizer.getPosition());
        return false;
    }

    public void stop() {}
}
