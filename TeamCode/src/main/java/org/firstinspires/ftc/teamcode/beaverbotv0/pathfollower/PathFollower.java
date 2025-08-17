package org.firstinspires.ftc.teamcode.beaverbotv0.pathfollower;

import org.firstinspires.ftc.teamcode.beaverbotv0.Resource;
import org.firstinspires.ftc.teamcode.beaverbotv0.Subsystem;
import org.firstinspires.ftc.teamcode.beaverbotv0.utils.PID;

import java.util.HashMap;
import java.util.Map;

public final class PathFollower<DOF extends Enum<DOF>, Movement extends Subsystem> extends Resource {
    public enum State {
        INCOMPLETE,
        COMPLETE,
    }

    private final Map<DOF, Path> path;
    private final Map<DOF, PID> pid;
    private final Map<DOF, Feedforward> feedforward;

    private final Map<DOF, Double> position = new HashMap<>();
    private final Map<DOF, Double> gradient = new HashMap<>();

    public PathFollower(Map<DOF, Path> path, Map<DOF, PID> pid, Map<DOF, Feedforward> feedforward) {
        this.path = path;
        this.pid = pid;
        this.feedforward = feedforward;
    }

    public State update(double dt, Map<DOF, Double> currentPosition) {
        path.entrySet().removeIf(entry -> {
            final DOF dof = entry.getKey();
            final Path pathAxis = entry.getValue();

            if (pathAxis.tick(dt) == Path.State.COMPLETE) {
                pid.remove(dof);
                feedforward.remove(dof);
                return true;
            }
            return false;
        });

        position.clear();
        gradient.clear();

        if (path.isEmpty()) {
            return State.COMPLETE;
        }

        path.forEach((dof, pathAxis) -> {
            position.put(dof, pathAxis.getPosition());

            final double error = pathAxis.getPosition() - currentPosition.get(dof);

            final double correction = pid.get(dof).correction(error, dt);
            final double prediction = feedforward.get(dof).prediction(
                    pathAxis.getVelocity(),
                    pathAxis.getAcceleration()
            );

            gradient.put(dof, correction + prediction);
        });

        return State.INCOMPLETE;
    }

    public Map<DOF, Double> getPosition() {
        return position;
    }

    public Map<DOF, Double> getGradient() {
        return gradient;
    }
}