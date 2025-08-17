package org.firstinspires.ftc.teamcode.pathfollowerv3;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Function;
import java.util.stream.DoubleStream;

public class PathBuilder {
    public ArrayList<PathSegment> pathSegments = new ArrayList<>();
    public Runnable onInit;
    public Runnable onIteration;
    public Runnable onInitBlocking;
    public HashMap<Robot.DOF, Function<Double, Double>> f;
    public Function<Double, Boolean> isFinished;
    public double clock = 0;
    public boolean autoIsFinished = true;

    public Robot.DOF[] dofs;

    public PathBuilder linearTo(HashMap<Robot.DOF, Double> points, double time) {
        for (Robot.DOF dof : dofs) {
            this.linearTo(dof, points.get(dof), time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder linearTo(Robot.DOF dof, double point, double time) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> function = f.get(dof);
        return this.append(dof, (Double t) -> (point - function.apply(startTimeCaptured)) * (t - startTimeCaptured) / time + function.apply(startTimeCaptured));
    }

    public PathBuilder easeCompoundPolynomialBezierTo(HashMap<Robot.DOF, double[]> points, double initialDegree, double finalDegree, double interpolationDegree, double time) {
        for (Robot.DOF dof : dofs) {
            this.easeCompoundPolynomialBezierTo(dof, points.get(dof), initialDegree, finalDegree, interpolationDegree, time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder easeCompoundPolynomialBezierTo(Robot.DOF dof, double[] points, double initialDegree, double finalDegree, double interpolationDegree, double time) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> function = f.get(dof);
        return this.append(dof, (Double t) -> MathUtils.bezier(
                DoubleStream.concat(
                        Arrays.stream(new double[] {function.apply(startTimeCaptured)}),
                        Arrays.stream(points)).toArray(),
                MathUtils.easeCompoundPolynomial(0, 1, initialDegree, finalDegree, interpolationDegree, t/time)));
    }

    public PathBuilder easePolynomialBezierTo(HashMap<Robot.DOF, double[]> points, double degree, double time) {
        for (Robot.DOF dof : dofs) {
            this.easePolynomialBezierTo(dof, points.get(dof), degree, time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder easePolynomialBezierTo(Robot.DOF dof, double[] points, double degree, double time) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> function = f.get(dof);
        return this.append(dof, (Double t) -> MathUtils.bezier(
                DoubleStream.concat(
                        Arrays.stream(new double[] {function.apply(startTimeCaptured)}),
                        Arrays.stream(points)).toArray(),
                MathUtils.easePolynomial(0, 1, degree, t/time)));
    }

    public PathBuilder easePolynomialTo(HashMap<Robot.DOF, Double> points, double degree, double time) {
        for (Robot.DOF dof : dofs) {
            this.easePolynomialTo(dof, points.get(dof), degree, time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder easePolynomialTo(Robot.DOF dof, double point, double degree, double time) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> function = f.get(dof);
        return this.append(dof, (Double t) -> MathUtils.easePolynomial(function.apply(startTimeCaptured), point, degree, t / time));
    }

    public PathBuilder bezierTo(HashMap<Robot.DOF, double[]> points, double time) {
        for (Robot.DOF dof : dofs) {
            this.bezierTo(dof, points.get(dof), time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder bezierTo(Robot.DOF dof, double[] points, double time) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> function = f.get(dof);
        return this.append(dof, (Double t) -> MathUtils.bezier(
                DoubleStream.concat(
                        Arrays.stream(new double[] {function.apply(startTimeCaptured)}),
                        Arrays.stream(points)).toArray(),
                t/time));
    }

    public PathBuilder easeCompoundPolynomialTo(HashMap<Robot.DOF, Double> points, double initialDegree, double finalDegree, double interpolationDegree, double time) {
        for (Robot.DOF dof : dofs) {
            this.easeCompoundPolynomialTo(dof, points.get(dof), initialDegree, finalDegree, interpolationDegree, time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder easeCompoundPolynomialTo(Robot.DOF dof, double point, double initialDegree, double finalDegree, double interpolationDegree, double time) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> function = f.get(dof);
        return this.append(dof, (Double t) -> MathUtils.easeCompoundPolynomial(function.apply(startTimeCaptured), point, initialDegree, finalDegree, interpolationDegree, t / time));
    }


    public PathBuilder followSubdivisions(HashMap<Robot.DOF, Double[]> points, double time) {
        for (Robot.DOF dof : dofs) {
            this.followSubdivisions(dof,
                    Arrays.stream(points.get(dof)).mapToDouble(Double::doubleValue).toArray(),
                    time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder followSubdivisions(Robot.DOF dof, double[] points, double time) {
        return this.append(dof, (Double t) -> MathUtils.interpolate(points, t, time));
    }

    public PathBuilder append(Robot.DOF dof, Function<Double, Double> f) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> previousFunctionCaptured = this.f.get(dof);
        return this.function(dof, (Double t) -> {
            if (t >= startTimeCaptured) return f.apply(t - startTimeCaptured);
            return previousFunctionCaptured.apply(t);
        });
    }

    public PathBuilder constify() {
        for (Robot.DOF dof : dofs) {
            this.constify(dof);
        }
        return this;
    }

    public PathBuilder constify(Robot.DOF dof) {
        double nCaptured = this.f.get(dof).apply(this.clock);
        return this.append(dof, (Double t) -> nCaptured);
    }

    public PathBuilder startingPoint(HashMap<Robot.DOF, Double> positions) {
        for (Robot.DOF dof : dofs) {
            this.startingPoint(dof, positions.get(dof));
        }
        return this;
    }

    public PathBuilder startingPoint(Robot.DOF dof, double position) {
        f.put(dof, (Double t) -> position);
        return this;
    }

    public PathBuilder addTime(double time) {
        return this.time(this.clock + time);
    }

    public PathBuilder time(double time) {
        this.clock = time;
        return this;
    }

    public PathBuilder function(Robot.DOF dof, Function<Double, Double> f) {
        this.f.put(dof, f);
        return this;
    }

    public PathBuilder isFinished(Function<Double, Boolean> isFinished) {
        this.autoIsFinished = false;
        this.isFinished = isFinished;
        return this;
    }

    public PathBuilder onInit(Runnable onInit) {
        this.onInit = onInit;
        return this;
    }

    public PathBuilder onIteration(Runnable onIteration) {
        this.onIteration = onIteration;
        return this;
    }


    public PathBuilder onInitBlocking(Runnable onInitBlocking) {
        this.onInitBlocking = onInitBlocking;
        return this;
    }

    public PathBuilder buildSegment() {
        if (autoIsFinished) {
            double capturedClock = this.clock;
            this.isFinished((Double t) -> t > capturedClock);
        }

        Function<Double, Boolean> isFinishedCaptured = isFinished;
        pathSegments.add(new PathSegment(f, isFinishedCaptured, onInit, onIteration, onInitBlocking));
        resetPathSegment();
        return this;
    }

    public Path build() {
        return new Path(pathSegments);
    }

    private void resetPathSegment() {
        this.f = new HashMap<Robot.DOF, Function<Double, Double>>() {
            {
                for (Robot.DOF dof : dofs) {
                    put(dof, (Double t) -> 0.0);
                }
            }
        };
        this.isFinished = (Double t) -> false;
        this.onInit = () -> {};
        this.onIteration = () -> {};
        this.onInitBlocking = () -> {};
        this.clock = 0;
        this.autoIsFinished = true;
    }

    public PathBuilder(Robot.DOF[] dofs) {
        // this.pathSegments = new ArrayList<>();
        this.dofs = dofs;
        resetPathSegment();
    }
}