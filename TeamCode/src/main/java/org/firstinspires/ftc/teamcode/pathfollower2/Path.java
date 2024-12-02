package org.firstinspires.ftc.teamcode.pathfollower2;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

public class Path {
    private final double epsilon = 1e-2;

    public int index = 0;
    public ArrayList<PathComponent> paths;
    public double t = 0;

    public boolean update(double dt) {
        t += dt;
        if (paths.get(index).isFinished.apply(t)) {
            index++;
            t = 0;
        }

        return index >= paths.size(); // true if the path is completed
    }

    public HashMap<DOFs.DOF, Double> getGradient() {
        return paths.get(index).getPositions.entrySet().stream().collect(
                HashMap::new,
                (HashMap<DOFs.DOF, Double> map, Map.Entry<DOFs.DOF, Function<Double, Double>> entry) -> map.put(entry.getKey(), (entry.getValue().apply(t + epsilon) - entry.getValue().apply(t)) / epsilon),
                HashMap::putAll);
    }

    public HashMap<DOFs.DOF, Double> getDeviation(HashMap<DOFs.DOF, Double> position) {
        return paths.get(index).getPositions.entrySet().stream().collect(
                HashMap::new,
                (HashMap<DOFs.DOF, Double> map, Map.Entry<DOFs.DOF, Function<Double, Double>> entry) -> map.put(entry.getKey(), position.get(entry.getKey()) - entry.getValue().apply(t)),
                HashMap::putAll);
    }

    public Path(ArrayList<PathComponent> paths) {
        this.paths = paths;
    }
}
