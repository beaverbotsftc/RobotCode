package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.PathAxis;
import org.beaverbots.beaver.pathing.pidf.PIDF;
import org.beaverbots.beaver.pathing.pidf.PIDFAxis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.List;
import java.util.Set;

public class AimAndResist implements Command {
    private Localizer localizer;
    private Drivetrain drivetrain;

    private Side side;
    private Command command;

    private boolean aim;

    public AimAndResist(Localizer localization, Drivetrain drivetrain, Side side, boolean aim) {
        this.localizer = localization;
        this.drivetrain = drivetrain;
        this.side = side;
        this.aim = aim;
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(drivetrain);
    }

    public void start() {
        DrivetrainState goalPosition = new DrivetrainState(Constants.GOAL_X, side == Side.RED ? Constants.GOAL_Y : -Constants.GOAL_Y, 0);
        DrivetrainState position = localizer.getPosition();

        command = new HolonomicFollowPath(
                new Path(
                        List.of(
                                new PathAxis(t -> position.getX(), 0, Double.POSITIVE_INFINITY),
                                new PathAxis(t -> position.getY(), 0, Double.POSITIVE_INFINITY),
                                new PathAxis(t ->
                                        aim
                                                ? Localizer.wind(localizer.getPosition().angleTo(goalPosition) - Constants.shooterBias, localizer.getPosition().getTheta())
                                                : position.getTheta(), 0, Double.POSITIVE_INFINITY)
                        ),
                        t -> false
                ),
                new PIDF(List.of(
                        new PIDFAxis(new PIDFAxis.K(1.5 * Constants.pidPX, Constants.pidIX, Constants.pidDX, 0, 6, 48, Constants.pidTauX, Constants.pidGammaX)),
                        new PIDFAxis(new PIDFAxis.K(1.5 * Constants.pidPY, Constants.pidIY, Constants.pidDY, 0, 6, 48, Constants.pidTauY, Constants.pidGammaY)),
                        new PIDFAxis(new PIDFAxis.K(2 * Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 0, 6, 48, Constants.pidTauTheta, Constants.pidGammaTheta))
                )),
                localizer, drivetrain
        );

        command.start();
    }

    public boolean periodic() {
        command.periodic();
        return false;
    }

    public void stop() {
        command.stop();
    }
}
