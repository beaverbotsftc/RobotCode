package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.Subsystem;
import org.beaverbots.beavertracking.HolonomicFollowPath;
import org.beaverbots.beavertracking.PIDF;
import org.beaverbots.beavertracking.PIDFAxis;
import org.beaverbots.beavertracking.Path;
import org.beaverbots.beavertracking.PathAxis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;

import java.util.List;
import java.util.Set;

public class Resist implements Command {
    private Pinpoint pinpoint;
    private Drivetrain drivetrain;

    private HolonomicFollowPath followPathCommand;
    private DrivetrainState target;

    public Resist(Pinpoint pinpoint, Drivetrain drivetrain) {
        this.pinpoint = pinpoint;
        this.drivetrain = drivetrain;
    }

    public Set<Subsystem> getDependencies() {
        return followPathCommand.getDependencies();
    }

    public void start() {
        DrivetrainState position = pinpoint.getPosition();
        followPathCommand = new HolonomicFollowPath(
                new Path(List.of(
                        new PathAxis(t -> position.getX(), 0, Double.POSITIVE_INFINITY),
                        new PathAxis(t -> position.getY(), 0, Double.POSITIVE_INFINITY),
                        new PathAxis(t -> position.getTheta(), 0, Double.POSITIVE_INFINITY)
                ), t -> false),
                new PIDF(List.of(
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, Constants.pidTauX)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, Constants.pidTauY)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, Constants.pidTauTheta))
                )),
                pinpoint, drivetrain
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
