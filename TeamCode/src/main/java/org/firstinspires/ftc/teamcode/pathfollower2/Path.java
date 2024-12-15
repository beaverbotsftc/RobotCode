package org.firstinspires.ftc.teamcode.pathfollower2;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.Function;

public class Path {
    public static class PathBuilder {
        public ArrayList<PathSegment> pathSegments;
        public HashMap<DOFs.DOF, Function<Double, Double>> f;
        public Function<Double, Boolean> isFinished;
        public double clock = 0;
        public boolean autoIsFinished = true;

        Telemetry telemetry;

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
            return this.append(dof, (Double t) -> Geometry.interpolate(points, t / time));
        }

        public PathBuilder append(DOFs.DOF dof, Function<Double, Double> f) {
            return this.function(dof,
                    ((BiFunction<Double, Function<Double, Double>, Function<Double, Double>>) ((Double startTime,
                                                                                                Function<Double, Double> previousFunction) -> (Double t) -> {
                        if (t >= startTime) {
                            return f.apply(t + startTime);
                        }
                        return previousFunction.apply(t);
                    })).apply(this.clock, this.f.get(dof)));
        }

        public PathBuilder constify() {
            for (DOFs.DOF dof : DOFs.DOF.values()) {
                this.constify(dof);
            }
            return this;
        }

        public PathBuilder constify(double time) {
            for (DOFs.DOF dof : DOFs.DOF.values()) {
                this.constify(dof, time);
            }
            return this;
        }

        public PathBuilder constify(DOFs.DOF dof) {
            return this.constify(dof, this.clock);
        }

        public PathBuilder constify(DOFs.DOF dof, double time) {
            telemetry.addData("B", this.f.get(dof).apply(this.clock));
            return this.append(dof, (Double t) -> this.f.get(dof).apply(this.clock));
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

        public PathBuilder buildSegment() {
            if (autoIsFinished) {
                this.isFinished(((Function<Double, Function<Double, Boolean>>) (Double endTime) -> (Double t) -> t > endTime)
                        .apply(this.clock));
            }
            pathSegments.add(new PathSegment(f, isFinished));
            resetPathSegment();
            return this;
        }

        public Path build() {
            return new Path(pathSegments);
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
            this.clock = 0;
            this.autoIsFinished = true;
        }

        public PathBuilder(Telemetry telemetry) {
            this.telemetry = telemetry;
            this.pathSegments = new ArrayList<>();
            resetPathSegment();
        }
    }

    private final double epsilon = 1e-3;

    public int index = 0;
    public ArrayList<PathSegment> paths;
    public double t = 0;

    public boolean update(double dt) {
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
