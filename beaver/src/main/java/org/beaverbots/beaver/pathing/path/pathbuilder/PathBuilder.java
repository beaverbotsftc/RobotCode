package org.beaverbots.beaver.pathing.path.pathbuilder;

import android.util.Pair;

import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.PathAxis;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.function.ToDoubleFunction;

public class PathBuilder {
    private static final double EPSILON = 1e-3;

    public enum EaseMode {
        DELAYED,
        PREEMPTIVE,
        CENTERED
    }

    protected double clock;
    protected List<DoubleUnaryOperator> f = new ArrayList<>();
    protected List<DoubleUnaryOperator> mirror = null;

    public PathBuilder(List<Double> startingPosition) {
        for (int i = 0; i < startingPosition.size(); i++) {
            final double captured = startingPosition.get(i);
            f.add(t -> captured);
        }
    }

    public PathBuilder(List<Double> startingPosition, List<DoubleUnaryOperator> mirror, boolean doubleMirrorStart) {
        this.mirror = mirror;
        for (int i = 0; i < startingPosition.size(); i++) {
            final double captured = doubleMirrorStart ? mirror.get(i).applyAsDouble(startingPosition.get(i)) : startingPosition.get(i);
            f.add(t -> captured);
        }
    }

    public PathBuilder(Path path) {
        this.f = new ArrayList<>();
        this.clock = 0.0;

        for (int i = 0; i < path.getAxes().size(); i++) {
            f.add(t -> 0.0);
        }

        appendPath(path);
    }

    public PathBuilder(Path path,
                       List<DoubleUnaryOperator> mirror,
                       boolean doubleMirrorStart) {
        this.mirror = mirror;
        this.f = new ArrayList<>();
        this.clock = 0.0;

        for (int i = 0; i < path.getAxes().size(); i++) {
            f.add(t -> 0.0);
        }

        if (doubleMirrorStart) {
            List<PathAxis> mirrored = new ArrayList<>();

            for (int i = 0; i < path.getAxes().size(); i++) {
                PathAxis axis = path.getAxes().get(i);
                DoubleUnaryOperator mirrorAxis = mirror.get(i);

                mirrored.add(new PathAxis(
                        t -> mirrorAxis.applyAsDouble(axis.getPath().applyAsDouble(t)),
                        axis.getStartTime(),
                        axis.getEndTime()
                ));
            }

            appendPath(new Path(mirrored, path::isFinished));
        } else {
            appendPath(path);
        }
    }

    public PathBuilder waitFor(double time) {
        clock += time;
        return this;
    }

    public PathBuilder appendPath(Path path) {
        double startTime = this.clock;
        double maxEndTime = 0.0;

        for (int i = 0; i < path.getAxes().size(); i++) {
            final PathAxis axis = path.getAxes().get(i);
            final DoubleUnaryOperator axisPath = axis.getPath();
            final DoubleUnaryOperator previous = f.get(i);
            final double axisStart = startTime;

            f.set(i, t -> {
                if (t < axisStart) {
                    return previous.applyAsDouble(t);
                }
                return axisPath.applyAsDouble(t - axisStart);
            });

            maxEndTime = Math.max(maxEndTime, axis.getEndTime());
        }

        this.clock += maxEndTime;

        return this;
    }

    private void append(int i, DoubleUnaryOperator fX, double time) {
        final DoubleUnaryOperator fPrevious = f.get(i);
        final double clockCaptured = clock;
        f.set(i, t -> t > clockCaptured ? fX.applyAsDouble(t - clockCaptured) : fPrevious.applyAsDouble(t));
    }


    public PathBuilder stop(double easingTime, double time) {
        // Default to PREEMPTIVE
        return stop(easingTime, time, EaseMode.PREEMPTIVE);
    }

    public PathBuilder stop(double easingTime, double time, EaseMode mode) {
        for (int i = 0; i < f.size(); i++) {
            final double targetCaptured = f.get(i).applyAsDouble(clock);
            appendEase(i, t -> targetCaptured, easingTime, mode);
        }

        clock += time;
        return this;
    }

    public PathBuilder stop(double time) {
        return this.stop(0, time);
    }

    public PathBuilder moveTo(List<Double> x, double time) {
        for (int i = 0; i < x.size(); i++) {
            final double targetCaptured = x.get(i);
            // Default to DELAYED
            appendEase(i, t -> targetCaptured, time, EaseMode.DELAYED);
        }
        clock += time;
        return this;
    }

    public PathBuilder linearTo(List<Double> x, double easingTime, double time) {
        // Default to DELAYED
        return linearTo(x, easingTime, time, EaseMode.DELAYED);
    }

    public PathBuilder linearTo(List<Double> x, double easingTime, double time, EaseMode mode) {
        for (int i = 0; i < x.size(); i++) {
            final DoubleUnaryOperator fPrevious = f.get(i);
            final double target = x.get(i);
            final double start = fPrevious.applyAsDouble(clock);

            DoubleUnaryOperator fNew = t -> {
                if (t <= 0) return start;
                double u = t / time;
                return start + u * (target - start);
            };

            appendEase(i, fNew, easingTime, mode);
        }
        clock += time;
        return this;
    }

    public PathBuilder linearTo(List<Double> x, double time) {
        return this.linearTo(x, 0, time);
    }

    public PathBuilder bezierTo(List<Double> control1, List<Double> control2, List<Double> target, double easingTime, double time) {
        // Default to DELAYED
        return bezierTo(control1, control2, target, easingTime, time, EaseMode.DELAYED);
    }

    public PathBuilder bezierTo(List<Double> control1, List<Double> control2, List<Double> target, double easingTime, double time, EaseMode mode) {
        for (int i = 0; i < f.size(); i++) {
            final DoubleUnaryOperator fPrevious = f.get(i);

            final double p0 = fPrevious.applyAsDouble(clock);
            final double p1 = control1.get(i);
            final double p2 = control2.get(i);
            final double p3 = target.get(i);

            DoubleUnaryOperator fNew = t -> {
                if (t <= 0) return p0;
                double u = t / time;
                if (u >= 1.0) return p3;

                double invU = 1.0 - u;
                double invU2 = invU * invU;
                double invU3 = invU2 * invU;
                double u2 = u * u;
                double u3 = u2 * u;

                return (invU3 * p0)
                        + (3 * invU2 * u * p1)
                        + (3 * invU * u2 * p2)
                        + (u3 * p3);
            };

            appendEase(i, fNew, easingTime, mode);
        }

        clock += time;
        return this;
    }

    public PathBuilder bezierTo(List<Double> control1, List<Double> control2, List<Double> target, double time) {
        return this.bezierTo(control1, control2, target, 0, time);
    }

    private void appendEase(int i, DoubleUnaryOperator fNew, double easingTime) {
        appendEase(i, fNew, easingTime, EaseMode.DELAYED);
    }

    private void appendEase(int i, DoubleUnaryOperator fNew, double easingTime, EaseMode mode) {
        final DoubleUnaryOperator fPrevious = f.get(i);
        final double waypointTime = clock;

        // Determine where the easing window starts based on the mode
        double transitionStart;
        switch (mode) {
            case PREEMPTIVE:
                transitionStart = waypointTime - easingTime;
                break;
            case CENTERED:
                transitionStart = waypointTime - (easingTime / 2.0);
                break;
            case DELAYED:
            default:
                transitionStart = waypointTime;
                break;
        }

        // fNew is still generated relative to the waypointTime (local t=0 is at clock),
        // but the blend transition is shifted in global time based on transitionStart.
        f.set(i, easeTransition(fPrevious, t -> fNew.applyAsDouble(t - waypointTime), transitionStart, easingTime));
    }

    private DoubleUnaryOperator easeTransition(
            DoubleUnaryOperator f0,
            DoubleUnaryOperator f1,
            double c,
            double easingTime
    ) {
        final DoubleUnaryOperator f0Safe = t -> f0.applyAsDouble(Math.max(0.0, t));

        final double b = c + easingTime;

        return t -> {
            // Before the transition window
            if (t <= c) {
                return f0Safe.applyAsDouble(t);
            }

            // Inside the transition window
            if (t < b) {
                double u = (t - c) / easingTime;
                double s = cubicSmoothstep(u);

                // Smooth continuation from the left (using Taylor series of f0)
                double yTaylor = taylor(f0Safe, c, t);

                // Future (target) path
                double y1 = f1.applyAsDouble(t);

                // Blend them
                return (1 - s) * yTaylor + s * y1;
            }

            // After easing window
            return f1.applyAsDouble(t);
        };
    }

    public PathBuilder reverse() {
        final double clockCaptured = this.clock;

        for (int i = 0; i < f.size(); i++) {
            final DoubleUnaryOperator fOriginal = f.get(i);
            f.set(i, t -> fOriginal.applyAsDouble(clockCaptured - t));
        }

        return this;
    }

    ///  Takes in a pair of the current position and the current velocity (both field centric)
    public PathBuilder retime(ToDoubleFunction<Pair<List<Double>, List<Double>>> usageRatio, double fraction, int samples) {
        final List<DoubleUnaryOperator> fOriginal = new ArrayList<>(this.f);
        final int dimensions = fOriginal.size();
        final double dtOriginal = this.clock / samples;

        List<Double> state = new ArrayList<>(dimensions);
        List<Double> velocity = new ArrayList<>(dimensions);
        for (int k = 0; k < dimensions; k++) {
            state.add(0.0);
            velocity.add(0.0);
        }

        double maxOriginalUsage = 0.0;

        for (int i = 1; i <= samples; i++) {
            double tNow = i * dtOriginal;
            double tPrevious = tNow - EPSILON;

            for (int k = 0; k < dimensions; k++) {
                double valueNow = fOriginal.get(k).applyAsDouble(tNow);
                double valuePrevious = fOriginal.get(k).applyAsDouble(tPrevious);

                state.set(k, valueNow);
                velocity.set(k, (valueNow - valuePrevious) / EPSILON);
            }

            double currentUsage = usageRatio.applyAsDouble(new Pair<>(state, velocity));

            if (currentUsage > maxOriginalUsage) {
                maxOriginalUsage = currentUsage;
            }
        }

        double scalingFactor = (maxOriginalUsage < EPSILON) ? 1.0 : (maxOriginalUsage / fraction);

        final double feasibleDuration = this.clock * scalingFactor;

        for (int i = 0; i < f.size(); i++) {
            final int iFinal = i;
            f.set(i, t -> fOriginal.get(iFinal).applyAsDouble(t / scalingFactor));
        }

        this.clock = feasibleDuration;

        return this;
    }


    public static ToDoubleFunction<Pair<List<Double>, List<Double>>> createHolonomicUsage(double maxVelX, double maxVelY, double maxVelTheta) {
        return pair -> {
            List<Double> s = pair.first;  // State
            List<Double> d = pair.second; // Derivative

            double vxField = d.get(0);
            double vyField = d.get(1);
            double vTheta = Math.abs(d.get(2));
            double heading = s.get(2);

            // Rotate Field-Centric velocities to Robot-Centric
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);
            double vxRobot = vxField * cosH + vyField * sinH;
            double vyRobot = -vxField * sinH + vyField * cosH;

            // Find the bottleneck
            double ratioX = Math.abs(vxRobot) / maxVelX;
            double ratioY = Math.abs(vyRobot) / maxVelY;
            double ratioTheta = vTheta / maxVelTheta;

            return Math.max(Math.max(ratioX, ratioY), ratioTheta);
        };
    }


    public Pair<Path, Path> build() {
        List<PathAxis> paths = new ArrayList<>();
        List<PathAxis> holdPaths = new ArrayList<>();
        for (int i = 0; i < f.size(); i++) {
            final DoubleUnaryOperator fAxis = f.get(i);
            final DoubleUnaryOperator mirrorAxis = mirror == null ? null : mirror.get(i);
            final DoubleUnaryOperator fAxisMirrored = mirrorAxis == null ? fAxis : t -> mirrorAxis.applyAsDouble(fAxis.applyAsDouble(t));
            paths.add(new PathAxis(fAxisMirrored, 0, clock));
            final double endpoint = fAxisMirrored.applyAsDouble(clock);
            holdPaths.add(new PathAxis(t -> endpoint, 0, Double.POSITIVE_INFINITY));
        }

        return new Pair<>(new Path(paths, t -> t >= clock), new Path(holdPaths, t -> false));
    }

    private double cubicSmoothstep(double t) {
        if (t <= 0) return 0.0;
        if (t >= 1) return 1.0;
        // 3x^2 - 2x^3
        return 3 * t * t - 2 * t * t * t;
    }

    /// 1st order
    private double taylor(DoubleUnaryOperator f, double c, double t) {
        double fc = f.applyAsDouble(c);
        double fc1 = f.applyAsDouble(c - EPSILON);
        double fc2 = f.applyAsDouble(c - 2 * EPSILON);

        double fp = (3 * fc - 4 * fc1 + fc2) / (2 * EPSILON);
        double fpp = (fc - 2 * fc1 + fc2) / (EPSILON * EPSILON);

        double dx = t - c;
        return fc + fp * dx;
    }
}