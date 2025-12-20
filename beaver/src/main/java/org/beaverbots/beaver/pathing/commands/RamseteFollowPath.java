package org.beaverbots.beaver.pathing.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pathing.Localizer;
import org.beaverbots.beaver.pathing.Locomotion;
import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.trackers.RamsetePathTracker;

import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public final class RamseteFollowPath implements Command  {
    private final Localizer localizer;
    private final Locomotion locomotion;

    private final RamsetePathTracker pathTracker;

    private ElapsedTime elapsedTime;


    public RamseteFollowPath(Path path, RamsetePathTracker.K k, Localizer localizer, Locomotion locomotion) {
        this.localizer = localizer;
        this.locomotion = locomotion;
        this.pathTracker = new RamsetePathTracker(path, k);
    }

    @Override
    public Set<Subsystem> getDependencies() {
        return new HashSet<>(Collections.singletonList(locomotion));
    }

    @Override
    public void start() {
        elapsedTime = new ElapsedTime();
    }

    @Override
    public boolean periodic() {
        final double dt = elapsedTime.seconds();
        if (dt == 0) return false;

        final List<Double> movement = pathTracker.update(localizer.getPositionAsList(), dt);

        locomotion.move(movement);

        elapsedTime.reset();

        return pathTracker.isFinished();
    }
}
