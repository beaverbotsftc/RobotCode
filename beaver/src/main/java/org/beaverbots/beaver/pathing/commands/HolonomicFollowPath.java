package org.beaverbots.beaver.pathing.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pathing.HolonomicPathTracker;
import org.beaverbots.beaver.pathing.Localizer;
import org.beaverbots.beaver.pathing.Locomotion;
import org.beaverbots.beaver.pathing.PIDF;
import org.beaverbots.beaver.pathing.Path;

import java.util.Collections;
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

        final List<Double> position = localizer.getPositionAsList();

        final List<Double> movement = pathTracker.update(position, dt);

        locomotion.move(movement, position);

        elapsedTime.reset();

        return pathTracker.isFinished();
    }
}
