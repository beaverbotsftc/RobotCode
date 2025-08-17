package org.firstinspires.ftc.teamcode.beaverbotv0.pathfollower;

import java.util.List;

public final class Path {
    public enum State {
        INCOMPLETE,
        COMPLETE,
    }

    private List<PathSegment> pathSegments;

    private int index = 0;

    public Path(List<PathSegment> pathSegments) {
        this.pathSegments = pathSegments;
    }

    public State tick(double dt) {
        if (pathSegments.get(index).tick(dt) == PathSegment.State.COMPLETE) {
            index += 1;
        }

        if (index == pathSegments.size()) return State.COMPLETE;
        return State.INCOMPLETE;
    }

    public double getPosition() {
        return pathSegments.get(index).getPosition();
    }

    public double getVelocity() {
        return pathSegments.get(index).getVelocity();
    }

    public double getAcceleration() {
        return pathSegments.get(index).getAcceleration();
    }
}
