package org.beaverbots.beavertracking;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public final class HolonomicPathTracker implements PathTracker {
    private final Path path;
    private final PIDF pidf;

    private final int dimensions;

    private double time = 0;

    public HolonomicPathTracker(Path path, PIDF pidf) {
        this.path = path;
        this.pidf = pidf;

        if (path.dimensions() != pidf.dimensions()) throw new IllegalArgumentException("Path and PIDF must be of the same dimensions");

        this.dimensions = path.dimensions();
    }

    public List<Double> update(List<Double> position, double dt) {
        if (dt <= 0) throw new IllegalArgumentException("dt must be positive (and non 0)");

        final List<Double> expectedPosition = path.position(time);
        final List<Double> expectedVelocity = path.velocity(time);

        final List<Double> error = IntStream.range(0, dimensions).mapToDouble(i -> expectedPosition.get(i) - position.get(i)).boxed().collect(Collectors.toList());

        time += dt;

        return pidf.update(error, expectedVelocity, dt);
    }

    public boolean isFinished() {
        return path.isFinished(time);
    }
}
