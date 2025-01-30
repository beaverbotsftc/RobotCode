package org.firstinspires.ftc.teamcode.pathfollower2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class Path {
    public static class PathBuilder {
        public ArrayList<PathSegment> pathSegments;
        public Consumer<Supplier<Boolean>> init;
        public HashMap<DOFs.DOF, Function<Double, Double>> f;
        public Function<Double, Boolean> isFinished;
        public double clock = 0;
        public boolean autoIsFinished = true;

        private Supplier<Boolean> isStopRequested;

        public PathBuilder linearTo(HashMap<DOFs.DOF, Double> points, double time) {
            for (DOFs.DOF dof : DOFs.DOF.values()) {
                this.linearTo(dof, points.get(dof), time);
            }
            return this.addTime(time).constify();
        }

        // Doesn't increase clock
        public PathBuilder linearTo(DOFs.DOF dof, double point, double time) {
            return this.followSubdivisions(dof, new double[]{f.get(dof).apply(this.clock), point}, time);
        }

        public PathBuilder followSubdivisions(HashMap<DOFs.DOF, Double[]> points, double time) {
            for (DOFs.DOF dof : DOFs.DOF.values()) {
                this.followSubdivisions(dof,
                        Arrays.stream(points.get(dof)).mapToDouble(Double::doubleValue).toArray(),
                        time);
            }
            return this.addTime(time).constify();
        }

        // Doesn't increase clock
        public PathBuilder followSubdivisions(DOFs.DOF dof, double[] points, double time) {
            return this.append(dof, (Double t) -> MathUtils.interpolate(points, t, time));
        }

        public PathBuilder append(DOFs.DOF dof, Function<Double, Double> f) {
            double startTimeCaptured = this.clock;
            Function<Double, Double> previousFunctionCaptured = this.f.get(dof);
            return this.function(dof, (Double t) -> {
                if (t >= startTimeCaptured) return f.apply(t - startTimeCaptured);
                return previousFunctionCaptured.apply(t);
            });
        }

        public PathBuilder constify() {
            for (DOFs.DOF dof : DOFs.DOF.values()) {
                this.constify(dof);
            }
            return this;
        }

        public PathBuilder constify(DOFs.DOF dof) {
            double nCaptured = this.f.get(dof).apply(this.clock);
            return this.append(dof, (Double t) -> nCaptured);
        }

        public PathBuilder addTime(double time) {
            return this.time(this.clock + time);
        }

        public PathBuilder time(double time) {
            this.clock = time;
            return this;
        }

        public PathBuilder function(DOFs.DOF dof, Function<Double, Double> f) {
            this.f.put(dof, f);
            return this;
        }

        public PathBuilder isFinished(Function<Double, Boolean> isFinished) {
            this.autoIsFinished = false;
            this.isFinished = isFinished;
            return this;
        }

        public PathBuilder init(Consumer<Supplier<Boolean>> init) {
            this.init = init;
            return this;
        }

        public PathBuilder buildSegment() {
            if (autoIsFinished) {
                double capturedClock = this.clock;
                this.isFinished((Double t) -> t > capturedClock);
            }
            pathSegments.add(new PathSegment(f, (Double t) -> (isFinished.apply(t) || isStopRequested.get()), init));
            resetPathSegment();
            return this;
        }

        public Path build() {
            return new Path(pathSegments, isStopRequested);
        }

        private void resetPathSegment() {
            this.f = new HashMap<DOFs.DOF, Function<Double, Double>>() {
                {
                    for (DOFs.DOF dof : DOFs.DOF.values()) {
                        put(dof, (Double t) -> 0.0);
                    }
                }
            };
            this.isFinished = (Double t) -> false;
            this.init = (Supplier<Boolean> isStopRequested) -> {};
            this.clock = 0;
            this.autoIsFinished = true;
        }

        public PathBuilder(Supplier<Boolean> isStopRequested) {
            this.pathSegments = new ArrayList<>();
            this.isStopRequested = isStopRequested;
            resetPathSegment();
        }
    }
    public int index = 0;
    private int lastIndex = -1; // -1 so that the init function for the first path gets run
    public ArrayList<PathSegment> paths;
    public double t = 0;

    private final double epsilon = 1e-3;
    private Supplier<Boolean> isStopRequested;

    public boolean update(double dt) {
        if (index != lastIndex) {
            new Thread(() -> paths.get(index).init.accept(this.isStopRequested)).start();
            lastIndex = index;
        }

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

    public Path(ArrayList<PathSegment> paths, Supplier<Boolean> isStopRequested) {
        this.paths = paths;
        this.isStopRequested = isStopRequested;
    }
}
