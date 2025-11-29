package org.beaverbots.beavertracking;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.Subsystem;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class HolonomicFollowPath implements Command  {
    private final Localizer localizer;
    private final Locomotion locomotion;

    private final HolonomicPathTracker pathTracker;

    private ElapsedTime elapsedTime;

    public HolonomicFollowPath(Path path, PIDF pidf, Localizer localizer, Locomotion locomotion) {
        this.localizer = localizer;
        this.locomotion = locomotion;
        this.pathTracker = new HolonomicPathTracker(path, pidf);
    }

    @Override
    public Set<Subsystem> getDependencies() {
        return new HashSet<>(Arrays.asList(localizer, locomotion));
    }

    @Override
    public void start() {
        elapsedTime = new ElapsedTime();
    }

    @Override
    public boolean periodic() {
        final double dt = elapsedTime.seconds();
        if (dt == 0) return false;

        final List<Double> position = localizer.getPositionAsList();

        final List<Double> movement = pathTracker.update(position, dt);

        locomotion.move(movement, position);

        elapsedTime.reset();

        return pathTracker.isFinished();
    }
}
