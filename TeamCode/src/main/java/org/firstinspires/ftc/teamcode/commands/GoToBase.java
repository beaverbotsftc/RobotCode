package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.PathAxis;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Modulus;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Transform;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.List;
import java.util.Set;

public class GoToBase implements Command {
    private Localizer localizer;
    private Drivetrain drivetrain;

    private Side side;
    private HolonomicFollowPath followPathCommand;

    public GoToBase(Localizer localization, Drivetrain drivetrain, Side side) {
        this.localizer = localization;
        this.drivetrain = drivetrain;
        this.side = side;
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(drivetrain);
    }

    public void start() {
        Transform position = localizer.getPosition();

        followPathCommand = new HolonomicFollowPath(
                /*
                new PathBuilder(position.toList())
                        .linearTo(new Transform(
                                Constants.BASE_X,
                                side == Side.RED ? Constants.BASE_Y : -Constants.BASE_Y,
                                position.getTheta() + Modulus.modulus(position.getTheta(), Math.PI / 2)).toList(),
                                1, 5)
                        .stop(1, 1)
                        .build()
                        .first
                 */
                // PID to point
                // Theta selection is suboptimal, but fine.
                new Path(List.of(
                        new PathAxis(t -> Constants.BASE_X, 0, Double.POSITIVE_INFINITY),
                        new PathAxis(t -> side == Side.RED ? Constants.BASE_Y : -Constants.BASE_Y, 0, Double.POSITIVE_INFINITY),
                        new PathAxis(t -> position.getTheta() + Modulus.modulus(position.getTheta(), Math.PI / 2), 0, Double.POSITIVE_INFINITY)
                ), t -> false),
                new PIDF(List.of(
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, Constants.pidTauX, Constants.pidGammaX)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, Constants.pidTauY, Constants.pidGammaY)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, Constants.pidTauTheta, Constants.pidGammaTheta))
                )),
                localizer, drivetrain
        );
        followPathCommand.start();
    }

    public boolean periodic() {
        followPathCommand.periodic();
        return false;
    }

    public void stop() {
        followPathCommand.stop();
    }
}
