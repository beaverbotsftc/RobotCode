package org.firstinspires.ftc.teamcode.beaverbotv0.pathfollower;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.DoublePredicate;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;
import java.util.stream.Collectors;

public final class PathGenerator<DOF extends Enum<DOF>> {
    private Set<DOF> dofs;

    final private Map<DOF, List<DoubleUnaryOperator>> path;
    final private Map<DOF, List<DoublePredicate>> finished;

    final private Map<DOF, Double> currentPose;
    private Map<DOF, Double> time;

    public PathGenerator appendTo(DOF dof, DoubleUnaryOperator f, double duration) {
        final int index = path.get(dof).size() - 1;

        final double capturedTime = time.get(dof);
        final DoubleUnaryOperator fPrevious = path.get(dof).get(index);

        path.get(dof).set(index,
                t -> t >= capturedTime
                        ? f.applyAsDouble(t - capturedTime)
                        : fPrevious.applyAsDouble(t)
        );

        time.put(dof, capturedTime + duration);
        currentPose.put(dof, f.applyAsDouble(duration));

        return this;
    }

    public Map<DOF, Path> build() {
        Map<DOF, Path> path = new HashMap<>();
        this.path.forEach((DOF dof, List<DoubleUnaryOperator> pathAxisRaw) -> {
            List<PathSegment> pathAxisSegments = new ArrayList<>();

            for (int i = 0; i < pathAxisRaw.size(); i++) {
                pathAxisSegments.add(new PathSegment(pathAxisRaw.get(i), finished.get(dof).get(i)));
            }

            path.put(dof, new Path(pathAxisSegments));
        });

        return path;
    }

    public PathGenerator(Map<DOF, Double> startingPose) {
        this.dofs = startingPose.keySet();
        this.currentPose = startingPose;
        this.time = dofs.stream().collect(Collectors.toMap(Function.identity(), dof -> 0.0));
        this.path = dofs.stream().collect(Collectors.toMap(Function.identity(), dof -> new ArrayList<>(
                Collections.singletonList(
                        t -> currentPose.get(dof)
                )
        )));
        this.finished = dofs.stream().collect(Collectors.toMap(Function.identity(), dof -> new ArrayList<>()));
    }
}
