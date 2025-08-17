package org.firstinspires.ftc.teamcode.pathfollowerv3;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class PathFollower { // extends Thread
    public static class K {
        public double gradientV;
        public double gradientA;
        public double pid;

        public K(double gradientV, double gradientA, double pid) {
            this.gradientV = gradientV;
            this.gradientA = gradientA;
            this.pid = pid;
        }
    }

    private Supplier<Boolean> isStopRequested;

    public Path path;

    public HashMap<Robot.DOF, PID> pids;
    public HashMap<Robot.DOF, K> k;

    public Robot.DOF[] dofs;

    public boolean finished = false;

    // Requires dt to be nonzero
    public HashMap<Robot.DOF, Double> tick(double dt, HashMap<Robot.DOF, Double> position) {
        if (finished) return null;

        if (path.tick(dt)) {
            finished = true;

            // Halt after all path segments are complete
            return new HashMap<Robot.DOF, Double>() {{
                for (Robot.DOF dof : dofs) {
                    put(dof, 0.0);
                }
            }};
        }

        HashMap<Robot.DOF, Double> gradientV = path.getGradientV();
        HashMap<Robot.DOF, Double> gradientA = path.getGradientA();
        HashMap<Robot.DOF, Double> deviation = path.getDeviation(position);

        for (Robot.DOF dof : dofs)
            pids.get(dof).tick(deviation.get(dof), dt);

        return pids.entrySet().stream().collect(HashMap::new,
                (HashMap<Robot.DOF, Double> map, Map.Entry<Robot.DOF, PID> entry)
                        -> map.put(entry.getKey(),
                            k.get(entry.getKey()).gradientV * gradientV.get(entry.getKey())
                            + k.get(entry.getKey()).gradientA * gradientA.get(entry.getKey())
                            + k.get(entry.getKey()).pid * pids.get(entry.getKey()).getCorrection()),
                HashMap::putAll);
    }

    public PathFollower(Robot.DOF[] dofs, Path path, HashMap<Robot.DOF, PID.K> pids, HashMap<Robot.DOF, K> k,
                        HashMap<Robot.DOF, Double> position, Supplier<Boolean> isStopRequested) {
        this.dofs = dofs;
        this.path = path;
        this.pids = pids.entrySet().stream().collect(HashMap::new,
                (HashMap<Robot.DOF, PID> map, Map.Entry<Robot.DOF, PID.K> entry)
                        -> map.put(entry.getKey(),
                        new PID(entry.getValue(), path.getDeviation(position).get(entry.getKey()))),
                HashMap::putAll);
        this.k = k;
        this.isStopRequested = isStopRequested;
    }
}