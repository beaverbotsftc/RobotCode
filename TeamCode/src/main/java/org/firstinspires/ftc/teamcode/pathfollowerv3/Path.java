package org.firstinspires.ftc.teamcode.pathfollowerv3;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

public class Path {
    public enum Status {
        Complete,
        Incomplete,
    }
    public int index = 0;
    public int lastIndex = -1;

    public ArrayList<PathSegment> paths;
    public double t = 0;

    private final double epsilon = 1e-3;

    public Utils.Pair<Status, Boolean> tick(double deltaTime) {
        t += deltaTime;

        if (paths.get(index).isFinished.apply(t)) {
            index++;
            t = 0;
        }

        final boolean updated = index != lastIndex;
        lastIndex = index;

        return new Utils.Pair<>(index >= paths.size() ? Status.Complete : Status.Incomplete, updated);
    }

    public HashMap<Robot.DOF, Double> getGradientV() {
        return paths.get(index).f.entrySet().stream().collect(
                HashMap::new,
                (HashMap<Robot.DOF, Double> map, Map.Entry<Robot.DOF, Function<Double, Double>> entry) -> map.put(entry.getKey(),
                        (entry.getValue().apply(t + epsilon) - entry.getValue().apply(t)) / epsilon
                ), HashMap::putAll);
    }

    public HashMap<Robot.DOF, Double> getGradientA() {
        return paths.get(index).f.entrySet().stream().collect(
                HashMap::new,
                (HashMap<Robot.DOF, Double> map, Map.Entry<Robot.DOF, Function<Double, Double>> entry) -> map.put(entry.getKey(),
                        (entry.getValue().apply(t) - 2 * entry.getValue().apply(t + epsilon) + entry.getValue().apply(t + 2 * epsilon)) / (epsilon * epsilon)
                ), HashMap::putAll);
    }

    public HashMap<Robot.DOF, Double> getDeviation(HashMap<Robot.DOF, Double> position) {
        return paths.get(index).f.entrySet().stream().collect(
                HashMap::new,
                (HashMap<Robot.DOF, Double> map, Map.Entry<Robot.DOF, Function<Double, Double>> entry) -> map.put(entry.getKey(),
                        position.get(entry.getKey()) - entry.getValue().apply(t)
                ), HashMap::putAll);
    }

    public Path(ArrayList<PathSegment> paths) {
        this.paths = paths;
    }
}
