package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.Subsystem;
import org.beaverbots.beavertracking.HolonomicFollowPath;
import org.beaverbots.beavertracking.PIDF;
import org.beaverbots.beavertracking.PIDFAxis;
import org.beaverbots.beavertracking.Path;
import org.beaverbots.beavertracking.PathAxis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.List;
import java.util.Set;

public class Resist implements Command {
    private Localizer localizer;
    private Drivetrain drivetrain;

    private Side side;
    private HolonomicFollowPath followPathCommand;
    private DrivetrainState target;

    public Resist(Localizer localization, Drivetrain drivetrain, Side side) {
        this.localizer = localization;
        this.drivetrain = drivetrain;
        this.side = side;
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(drivetrain);
    }

    public void start() {
        DrivetrainState position = localizer.getPosition();

        double desiredAngle = Double.NaN;
        switch (side) {
            case RED: desiredAngle = localizer.wind(Math.atan2(Constants.redGoalY - localizer.getPosition().getY(), Constants.redGoalX - localizer.getPosition().getX())); break;
            case BLUE: desiredAngle = localizer.wind(Math.atan2(Constants.blueGoalY - localizer.getPosition().getY(), Constants.blueGoalX - localizer.getPosition().getX())); break;
        }

        if (Math.abs(desiredAngle - position.getTheta()) > Math.PI / 2) desiredAngle = position.getTheta();

        final double finalDesiredAngle = desiredAngle;
        followPathCommand = new HolonomicFollowPath(
                new Path(List.of(
                        new PathAxis(t -> position.getX(), 0, Double.POSITIVE_INFINITY),
                        new PathAxis(t -> position.getY(), 0, Double.POSITIVE_INFINITY),
                        new PathAxis(t -> finalDesiredAngle, 0, Double.POSITIVE_INFINITY)
                ), t -> false),
                new PIDF(List.of(
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, Constants.pidTauX)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, Constants.pidTauY)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, Constants.pidTauTheta))
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
