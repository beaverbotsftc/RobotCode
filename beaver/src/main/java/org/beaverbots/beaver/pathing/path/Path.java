package org.beaverbots.beaver.pathing.path;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoublePredicate;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.Collectors;

public final class Path {
    private final List<PathAxis> path;
    private final DoublePredicate isFinishedPredicate;

    public Path(List<PathAxis> path, DoublePredicate isFinishedPredicate) {
        this.path = path;
        this.isFinishedPredicate = isFinishedPredicate;
    }

    public int getDimensions() {
        return path.size();
    }

    public boolean isFinished(double t) {
        return isFinishedPredicate.test(t);
    }

    public List<Double> position(double t) {
        return path.stream().map(x -> x.position(t)).collect(Collectors.toList());
    }

    public List<Double> velocity(double t) {
        return path.stream().map(pathAxis -> pathAxis.velocity(t)).collect(Collectors.toList());
    }

    public List<Double> acceleration(double t) {
        return path.stream().map(x -> x.acceleration(t)).collect(Collectors.toList());
    }

    ///  Please don't mutate the result!
    public List<PathAxis> getAxes() {
        return path;
    }

    public Path transform(List<DoubleUnaryOperator> f) {
        List<PathAxis> newPath = new ArrayList<>();
        for (int i = 0; i < getDimensions(); i++) {
            newPath.add(path.get(i).transform(f.get(i)));
        }
        return new Path(newPath, isFinishedPredicate);
    }
}
