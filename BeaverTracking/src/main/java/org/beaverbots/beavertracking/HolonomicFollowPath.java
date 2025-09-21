package org.beaverbots.beavertracking;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.Subsystem;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public final class HolonomicFollowPath implements Command  {
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

        final List<Double> movement = pathTracker.update(localizer.getPosition(), dt);

        locomotion.move(movement);

        elapsedTime.reset();

        return pathTracker.isFinished();
    }
}
