package org.firstinspires.ftc.teamcode.pathfollower2;

import java.util.HashMap;
import java.util.Map;

public class PathFollower implements Runnable {
    public static class PathFollowerCoefficients {
        public double gradient;
        public double pid;

        public PathFollowerCoefficients(double gradient, double pid) {
            this.gradient = gradient;
            this.pid = pid;
        }
    }

    public Path path;
    public DOFs dofs;
    public HashMap<DOFs.DOF, PID> pids;
    public HashMap<DOFs.DOF, PathFollowerCoefficients> k;

    private double lastLoopTime;

    public void run() {
        while (true) {
            double time = (double) System.currentTimeMillis() * 1e-3;
            double dt = lastLoopTime - time;
            if (dt == 0) continue; // Avoid division by zero
            lastLoopTime = time;

            if (path.update(dt)) { break; };

            HashMap<DOFs.DOF, Double> gradient = path.getGradient();
            HashMap<DOFs.DOF, Double> deviation = path.getDeviation(dofs.getPositions());

            for (DOFs.DOF dof : DOFs.DOF.values()) {
                pids.get(dof).update(deviation.get(dof), dt);
            }

            dofs.apply(
                    pids.entrySet().stream().collect(
                            HashMap::new,
                            (HashMap<DOFs.DOF, Double> map, Map.Entry<DOFs.DOF, PID> entry) -> map.put(entry.getKey(), k.get(entry.getKey()).gradient * gradient.get(entry.getKey()) + k.get(entry.getKey()).pid * pids.get(entry.getKey()).correction),
                            HashMap::putAll)
            );
        }
    }

    public PathFollower(Path path, DOFs dofs, HashMap<DOFs.DOF, PID.PIDCoefficients> pids, HashMap<DOFs.DOF, PathFollowerCoefficients> k) {
        this.path = path;
        this.dofs = dofs;
        this.pids = new HashMap<>();
        this.k = k;
        this.lastLoopTime = (double) System.currentTimeMillis() * 1e-3;

        HashMap<DOFs.DOF, Double> deviation = path.getDeviation(dofs.getPositions());
        for (DOFs.DOF dof : DOFs.DOF.values()) {
            this.pids.put(dof, new PID(deviation.get(dof), pids.get(dof)));
        }
    }
}
