package org.firstinspires.ftc.teamcode.pathfollower2;

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

    public boolean update(double dt) {
        if (index != lastIndex) {
            new Thread(paths.get(index).onInit).start();
            paths.get(index).onInitBlocking.run();
            lastIndex = index;
        }

        new Thread(paths.get(index).onIteration).start();
        paths.get(index).onIterationBlocking.run();

        t += dt;

        if (paths.get(index).isFinished.apply(t)) {
            index++;
            t = 0;
        }

        return index >= paths.size();
    }

    public HashMap<DOFs.DOF, Double> getGradient() {
        return paths.get(index).f.entrySet().stream().collect(
                HashMap::new,
                (HashMap<DOFs.DOF, Double> map, Map.Entry<DOFs.DOF, Function<Double, Double>> entry) -> map.put(entry.getKey(),
                        (entry.getValue().apply(t + epsilon) - entry.getValue().apply(t)) / epsilon),
                HashMap::putAll);
    }

    public HashMap<DOFs.DOF, Double> getDeviation(HashMap<DOFs.DOF, Double> position) {
        return paths.get(index).f.entrySet().stream().collect(
                HashMap::new,
                (HashMap<DOFs.DOF, Double> map, Map.Entry<DOFs.DOF, Function<Double, Double>> entry) -> map.put(entry.getKey(),
                        position.get(entry.getKey()) - entry.getValue().apply(t)),
                HashMap::putAll);
    }

    public Path(ArrayList<PathSegment> paths) {
        this.paths = paths;
    }
}
