package org.firstinspires.ftc.teamcode.pathfollower2;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class PathFollower { // extends Thread
    public static class K {
        public double gradient;
        public double pid;

        public K(double gradient, double pid) {
            this.gradient = gradient;
            this.pid = pid;
        }
    }

    private Supplier<Boolean> isStopRequested;

    public Path path;
    public DOFs dofs;
    public HashMap<DOFs.DOF, PID> pids;
    public HashMap<DOFs.DOF, K> k;

    public void run(Telemetry telemetry) {
        double lastLoopTime = (double) System.currentTimeMillis() * 1e-3;

        while (!isStopRequested.get()) {
            double time = (double) System.currentTimeMillis() * 1e-3;
            double dt = time - lastLoopTime;
            if (!(dt > 0))
                continue; // Prevent division by 0

            lastLoopTime = time;

            if (path.update(dt)) {
                dofs.apply(new HashMap<DOFs.DOF, Double>() {{
                    for (DOFs.DOF dof : DOFs.DOF.values()) {
                        put(dof, 0.0);
                    }
                }}, telemetry);
                break; // Stop once the path is complete
            }

            telemetry.addData("X", dofs.getPosition().get(DOFs.DOF.X));
            telemetry.addData("Y", dofs.getPosition().get(DOFs.DOF.Y));
            telemetry.addData("THETA", dofs.getPosition().get(DOFs.DOF.THETA));

            HashMap<DOFs.DOF, Double> gradient = path.getGradient();
            HashMap<DOFs.DOF, Double> deviation = path.getDeviation(dofs.getPosition());

            for (DOFs.DOF dof : DOFs.DOF.values()) {
                pids.get(dof).update(deviation.get(dof), dt);
            }

            telemetry.addData("pids", pids);

            dofs.apply(
                    pids.entrySet().stream().collect(
                            HashMap::new,
                            (HashMap<DOFs.DOF, Double> map, Map.Entry<DOFs.DOF, PID> entry) -> map.put(entry.getKey(),
                                    k.get(entry.getKey()).gradient * gradient.get(entry.getKey())
                                            + k.get(entry.getKey()).pid * pids.get(entry.getKey()).getCorrection()),
                            HashMap::putAll), telemetry);
            telemetry.update();
        }
    }

    public PathFollower(Path path, DOFs dofs, HashMap<DOFs.DOF, PID.K> pids, HashMap<DOFs.DOF, K> k, Supplier<Boolean> isStopRequested) {
        this.path = path;
        this.dofs = dofs;
        this.pids = pids.entrySet().stream().collect(
                HashMap::new, (HashMap<DOFs.DOF, PID> map, Map.Entry<DOFs.DOF, PID.K> entry) -> map.put(entry.getKey(),
                        new PID(entry.getValue(), path.getDeviation(dofs.getPosition()).get(entry.getKey()))),
                HashMap::putAll);
        this.k = k;
        this.isStopRequested = isStopRequested;
    }
}
