package org.firstinspires.ftc.teamcode.pathfollowerv3;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

public class Path {
    public int index = 0;
    private int lastIndex = -1; // -1 so that the onInit function for the first path gets run
    public ArrayList<PathSegment> paths;
    public double t = 0;

    private final double epsilon = 1e-3;

    public boolean tick(double dt) {
        if (index != lastIndex) {
            new Thread(paths.get(index).onInit).start();
            paths.get(index).onInitBlocking.run();
            lastIndex = index;
        }

        paths.get(index).onTick.run();

        t += dt;

        if (paths.get(index).isFinished.apply(t)) {
            index++;
            t = 0;
        }

        return index >= paths.size();
    }

    public HashMap<Robot.DOF, Double> getGradientV() {
        return paths.get(index).f.entrySet().stream().collect(
                HashMap::new,
                (HashMap<Robot.DOF, Double> map, Map.Entry<Robot.DOF, Function<Double, Double>> entry) -> map.put(entry.getKey(),
                        (entry.getValue().apply(t + epsilon) - entry.getValue().apply(t)) / epsilon),
                HashMap::putAll);
    }

    public HashMap<Robot.DOF, Double> getGradientA() {
        return paths.get(index).f.entrySet().stream().collect(
                HashMap::new,
                (HashMap<Robot.DOF, Double> map, Map.Entry<Robot.DOF, Function<Double, Double>> entry) -> map.put(entry.getKey(),
                        (entry.getValue().apply(t) - 2 * entry.getValue().apply(t + epsilon) + entry.getValue().apply(t + 2 * epsilon)) / (epsilon * epsilon)),
                HashMap::putAll);
    }

    public HashMap<Robot.DOF, Double> getDeviation(HashMap<Robot.DOF, Double> position) {
        return paths.get(index).f.entrySet().stream().collect(
                HashMap::new,
                (HashMap<Robot.DOF, Double> map, Map.Entry<Robot.DOF, Function<Double, Double>> entry) -> map.put(entry.getKey(),
                        position.get(entry.getKey()) - entry.getValue().apply(t)),
                HashMap::putAll);
    }

    public Path(ArrayList<PathSegment> paths) {
        this.paths = paths;
    }
}
