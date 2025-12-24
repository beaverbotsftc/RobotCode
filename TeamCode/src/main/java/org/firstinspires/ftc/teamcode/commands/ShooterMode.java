package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.pidf.PIDF;
import org.beaverbots.beaver.pathing.pidf.PIDFAxis;
import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.PathAxis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.List;
import java.util.Set;

public class ShooterMode implements Command {
    private Localizer localizer;
    private Drivetrain drivetrain;

    private Side side;
    private HolonomicFollowPath followPathCommand;
    private DrivetrainState target;
    private boolean resistOnly;

    public ShooterMode(Localizer localization, Drivetrain drivetrain, Side side, boolean resistOnly) {
        this.localizer = localization;
        this.drivetrain = drivetrain;
        this.side = side;
        this.resistOnly = resistOnly;
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(drivetrain);
    }

    public void start() {
        DrivetrainState position = localizer.getPosition();

        double desiredAngle = Double.NaN;
        switch (side) {
            case RED: desiredAngle = localizer.wind(Math.atan2(Constants.GOAL_Y - localizer.getPosition().getY(), Constants.GOAL_X - localizer.getPosition().getX())); break;
            case BLUE: desiredAngle = localizer.wind(Math.atan2(-Constants.GOAL_Y - localizer.getPosition().getY(), Constants.GOAL_X- localizer.getPosition().getX())); break;
        }
        desiredAngle -= Constants.shooterBias; // TODO: Correction

        if (Math.abs(desiredAngle - position.getTheta()) > Math.PI / 2) desiredAngle = position.getTheta();

        final double finalDesiredAngle = desiredAngle;
        followPathCommand = new HolonomicFollowPath(
                new Path(List.of(
                        new PathAxis(t -> position.getX(), 0, Double.POSITIVE_INFINITY),
                        new PathAxis(t -> position.getY(), 0, Double.POSITIVE_INFINITY),
                        new PathAxis(t -> resistOnly ? position.getTheta() : finalDesiredAngle, 0, Double.POSITIVE_INFINITY)
                ), t -> false),
                new PIDF(List.of(
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPX * 1.5, Constants.pidIX, Constants.pidDX, 1, 6, 48, Constants.pidTauX, Constants.pidGammaX)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPY * 1.5, Constants.pidIY, Constants.pidDY, 1, 6, 48, Constants.pidTauY, Constants.pidGammaY)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta * 2, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, Constants.pidTauTheta, Constants.pidGammaTheta))
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
