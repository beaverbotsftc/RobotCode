package org.firstinspires.ftc.teamcode.pathfollower2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Function;
import java.util.stream.DoubleStream;

public class PathBuilder {
    public ArrayList<PathSegment> pathSegments;
    public Runnable onInit;
    public Runnable onIteration;
    public Runnable onInitBlocking;
    public Runnable onIterationBlocking;
    public HashMap<DOFs.DOF, Function<Double, Double>> f;
    public Function<Double, Boolean> isFinished;
    public double clock = 0;
    public boolean autoIsFinished = true;

    public PathBuilder linearTo(HashMap<DOFs.DOF, Double> points, double time) {
        for (DOFs.DOF dof : DOFs.DOF.values()) {
            this.linearTo(dof, points.get(dof), time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder linearTo(DOFs.DOF dof, double point, double time) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> function = f.get(dof);
        return this.append(dof, (Double t) -> (point - function.apply(startTimeCaptured)) * (t - startTimeCaptured) / time + function.apply(startTimeCaptured));
    }

    public PathBuilder easePolynomialBezierTo(HashMap<DOFs.DOF, double[]> points, double degree, double time) {
        for (DOFs.DOF dof : DOFs.DOF.values()) {
            this.easePolynomialBezierTo(dof, points.get(dof), degree, time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder easePolynomialBezierTo(DOFs.DOF dof, double[] points, double degree, double time) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> function = f.get(dof);
        return this.append(dof, (Double t) -> MathUtils.bezier(
                DoubleStream.concat(
                        Arrays.stream(new double[] {function.apply(startTimeCaptured)}),
                        Arrays.stream(points)).toArray(),
                MathUtils.easePolynomial(0, 1, degree, t/time)));
    }

    public PathBuilder easePolynomialTo(HashMap<DOFs.DOF, Double> points, double degree, double time) {
        for (DOFs.DOF dof : DOFs.DOF.values()) {
            this.easePolynomialTo(dof, points.get(dof), degree, time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder easePolynomialTo(DOFs.DOF dof, double point, double degree, double time) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> function = f.get(dof);
        return this.append(dof, (Double t) -> MathUtils.easePolynomial(function.apply(startTimeCaptured), point, degree, t / time));
    }

    public PathBuilder bezierTo(HashMap<DOFs.DOF, double[]> points, double time) {
        for (DOFs.DOF dof : DOFs.DOF.values()) {
            this.bezierTo(dof, points.get(dof), time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder bezierTo(DOFs.DOF dof, double[] points, double time) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> function = f.get(dof);
        return this.append(dof, (Double t) -> MathUtils.bezier(
                DoubleStream.concat(
                        Arrays.stream(new double[] {function.apply(startTimeCaptured)}),
                        Arrays.stream(points)).toArray(),
                t/time));
    }

    public PathBuilder easeCompoundPolynomialTo(HashMap<DOFs.DOF, Double> points, double initialDegree, double finalDegree, double interpolationDegree, double time) {
        for (DOFs.DOF dof : DOFs.DOF.values()) {
            this.easeCompoundPolynomialTo(dof, points.get(dof), initialDegree, finalDegree, interpolationDegree, time);
        }
        return this.addTime(time).constify();
    }

    // Doesn't increase clock
    public PathBuilder easeCompoundPolynomialTo(DOFs.DOF dof, double point, double initialDegree, double finalDegree, double interpolationDegree, double time) {
        double startTimeCaptured = this.clock;
        Function<Double, Double> function = f.get(dof);
        return this.append(dof, (Double t) -> MathUtils.easeCompoundPolynomial(function.apply(startTimeCaptured), point, initialDegree, finalDegree, interpolationDegree, t / time));
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

    public PathBuilder onIterationBlocking(Runnable onIterationBlocking) {
        this.onIterationBlocking = onIterationBlocking;
        return this;
    }

    public PathBuilder buildSegment() {
        if (autoIsFinished) {
            double capturedClock = this.clock;
            this.isFinished((Double t) -> t > capturedClock);
        }
        pathSegments.add(new PathSegment(f, (Double t) -> isFinished.apply(t), onInit, onIteration, onInitBlocking, onIterationBlocking));
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
        this.onInit = () -> {};
        this.onIteration = () -> {};
        this.onInitBlocking = () -> {};
        this.onIterationBlocking = () -> {};
        this.clock = 0;
        this.autoIsFinished = true;
    }

    public PathBuilder() {
        this.pathSegments = new ArrayList<>();
        resetPathSegment();
    }
}