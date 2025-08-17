package org.firstinspires.ftc.teamcode.pathfollowerv3;

import java.util.HashMap;
import java.util.Map;

public class PathFollower { // extends Thread
    public static class K {
        public double pid;
        public double fv;
        public double fa;
        public double c;

        public K(double pid, double fv, double fa, double c) {
            this.pid = pid;
            this.fv = fv;
            this.fa = fa;
            this.c = c;
        }
    }

    public enum Status {
        Complete,
        Incomplete,
    }

    public Path path;

    public HashMap<Robot.DOF, PID> pids;
    public HashMap<Robot.DOF, K> k;

    public Robot.DOF[] dofs;

    // Requires deltaTime to be nonzero
    public Utils.Ternary<HashMap<Robot.DOF, Double>, Status, Boolean> tick(double deltaTime, HashMap<Robot.DOF, Double> position) {
        Utils.Pair<Path.Status, Boolean> pathStatus = path.tick(deltaTime);
        if (pathStatus.x1 == Path.Status.Complete) {
            return new Utils.Ternary<>(new HashMap<Robot.DOF, Double>() {{
                for (Robot.DOF dof : dofs) {
                    put(dof, 0.0);
                }
            }}, Status.Complete, pathStatus.x2);
        }

        HashMap<Robot.DOF, Double> deviation = path.getDeviation(position);
        HashMap<Robot.DOF, Double> gradientV = path.getGradientV();
        HashMap<Robot.DOF, Double> gradientA = path.getGradientA();
        final double speed = gradientV.values().stream().mapToDouble(Double::doubleValue).sum();
        HashMap<Robot.DOF, Double> direction;
        if (!Utils.equals(speed, 0)) {
             direction = gradientV.entrySet().stream().collect(HashMap::new,
                    (HashMap<Robot.DOF, Double> map, Map.Entry<Robot.DOF, Double> entry)
                            -> map.put(entry.getKey(), gradientV.get(entry.getKey()) / speed),
                    HashMap::putAll);
        } else {
            direction = null;
        }

        for (Robot.DOF dof : dofs)
            pids.get(dof).tick(deviation.get(dof), deltaTime);

        return new Utils.Ternary<>(pids.entrySet().stream().collect(HashMap::new,
                (HashMap<Robot.DOF, Double> map, Map.Entry<Robot.DOF, PID> entry)
                        -> map.put(entry.getKey(),
                            k.get(entry.getKey()).pid * pids.get(entry.getKey()).getCorrection()
                            + k.get(entry.getKey()).fv * gradientV.get(entry.getKey())
                            + k.get(entry.getKey()).fa * gradientA.get(entry.getKey())
                            + (direction != null ? k.get(entry.getKey()).c * direction.get(entry.getKey()) : 0)), // Friction does not scale with speed
                HashMap::putAll), Status.Incomplete, pathStatus.x2);
    }

    public PathFollower(Robot.DOF[] dofs, Path path, HashMap<Robot.DOF, PID.K> pids, HashMap<Robot.DOF, K> k,
                        HashMap<Robot.DOF, Double> position) {
        this.dofs = dofs;
        this.path = path;
        this.pids = pids.entrySet().stream().collect(HashMap::new,
                (HashMap<Robot.DOF, PID> map, Map.Entry<Robot.DOF, PID.K> entry)
                        -> map.put(entry.getKey(),
                        new PID(entry.getValue(), path.getDeviation(position).get(entry.getKey()))),
                HashMap::putAll);
        this.k = k;
    }
}