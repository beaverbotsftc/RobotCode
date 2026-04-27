package org.beaverbots.beaver.pathing.path.pathbuilder;

import org.beaverbots.beaver.util.Pair;

import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.PathAxis;
import org.beaverbots.beaver.util.Triple;

import java.util.ArrayList;
import java.util.Collections;
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
            final PathAxis axis = path.getAxes().get(i);
            final double captured = axis.getPath().applyAsDouble(0.0);
            f.add(t -> captured);
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
            final PathAxis axis = path.getAxes().get(i);
            final double captured = doubleMirrorStart ? mirror.get(i).applyAsDouble(axis.getPath().applyAsDouble(0.0)) : axis.getPath().applyAsDouble(0.0);
            f.add(t -> captured);
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

    public PathBuilder bezierTo(List<List<Double>> controlPoints, double easingTime, double time, EaseMode mode) {
        for (int i = 0; i < f.size(); i++) {
            final DoubleUnaryOperator fPrevious = f.get(i);
            final double p0 = fPrevious.applyAsDouble(clock);

            // Build the full per-axis control point list [P0, P1, ..., Pn].
            final List<Double> axisPoints = new ArrayList<>(controlPoints.size() + 1);
            axisPoints.add(p0);
            for (List<Double> controlPoint : controlPoints) {
                axisPoints.add(controlPoint.get(i));
            }

            DoubleUnaryOperator fNew = t -> {
                if (t <= 0) return axisPoints.get(0);
                double u = t / time;
                if (u >= 1.0) return axisPoints.get(axisPoints.size() - 1);
                return deCasteljau(axisPoints, u);
            };

            appendEase(i, fNew, easingTime, mode);
        }

        clock += time;
        return this;
    }

    public PathBuilder bezierTo(List<List<Double>> controlPoints, double easingTime, double time) {
        return bezierTo(controlPoints, easingTime, time, EaseMode.DELAYED);
    }

    public PathBuilder bezierTo(List<List<Double>> controlPoints, double time) {
        return bezierTo(controlPoints, 0, time);
    }

    public PathBuilder c2BezierTo(List<List<Double>> additionalControlPoints, double time) {
        // Degree = 2 (for the two constrained control points) + user point count.
        // Total control points: P0 (implicit) + P1 (C1 constraint) + P2 (C2 constraint) + user points.
        int degree = 2 + additionalControlPoints.size();

        for (int i = 0; i < f.size(); i++) {
            final DoubleUnaryOperator fCurrent = f.get(i);

            // 2nd-order backward finite differences for velocity and acceleration at clock.
            // Uses the same three-point stencil as taylor() for consistency.
            double valueAtClock          = fCurrent.applyAsDouble(clock);
            double valueAtClockMinusEps  = fCurrent.applyAsDouble(clock - EPSILON);
            double valueAtClockMinus2Eps = fCurrent.applyAsDouble(clock - 2 * EPSILON);

            double velocity     = (3 * valueAtClock - 4 * valueAtClockMinusEps + valueAtClockMinus2Eps) / (2 * EPSILON);
            double acceleration = (valueAtClock - 2 * valueAtClockMinusEps + valueAtClockMinus2Eps) / (EPSILON * EPSILON);

            // Solve for P1 and P2 from the C1 and C2 junction conditions:
            //   B'(0)  = (n / T) * (P1 - P0)        = velocity
            //   B''(0) = (n*(n-1) / T^2) * (P2 - 2*P1 + P0) = acceleration
            double p0 = valueAtClock;
            double p1 = p0 + velocity * time / degree;
            double p2 = acceleration * time * time / ((double) degree * (degree - 1)) + 2 * p1 - p0;

            final List<Double> axisPoints = new ArrayList<>(degree + 1);
            axisPoints.add(p0);
            axisPoints.add(p1);
            axisPoints.add(p2);
            for (List<Double> controlPoint : additionalControlPoints) {
                axisPoints.add(controlPoint.get(i));
            }

            // Hard switch at clock — no easing window needed since C2 is enforced by construction.
            appendEase(i, t -> {
                if (t <= 0) return p0;
                double u = t / time;
                if (u >= 1.0) return axisPoints.get(axisPoints.size() - 1);
                return deCasteljau(axisPoints, u);
            }, 0, EaseMode.DELAYED);
        }

        clock += time;
        return this;
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
        f.set(i, easeTransition(fPrevious, t -> fNew.applyAsDouble(t - waypointTime), transitionStart, easingTime, waypointTime));
    }

    private DoubleUnaryOperator easeTransition(
            DoubleUnaryOperator f0,
            DoubleUnaryOperator f1,
            double c,
            double easingTime,
            double waypointTime
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
                double s = quinticSmoothstep(u);

                // Smooth continuation of f0 into the future
                double yTaylor = backwardTaylor(f0Safe, c, t);

                // Smooth backward projection of f1 into the past
                double y1;
                if (t < waypointTime) {
                    // Extracts the derivatives at t=waypointTime using only positive values
                    y1 = forwardTaylor(f1, waypointTime, t);
                } else {
                    y1 = f1.applyAsDouble(t);
                }

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
    public PathBuilder retime(ToDoubleFunction<Triple<List<Double>, List<Double>, List<Double>>> usageRatio, double fraction, int samples, boolean considerAcceleration) {
        final List<DoubleUnaryOperator> fOriginal = new ArrayList<>(this.f);
        final int dimensions = fOriginal.size();
        final double dtOriginal = this.clock / samples;

        List<Double> state = new ArrayList<>(dimensions);
        List<Double> velocity = new ArrayList<>(dimensions);
        List<Double> acceleration = new ArrayList<>(dimensions);
        for (int k = 0; k < dimensions; k++) {
            state.add(0.0);
            velocity.add(0.0);
            acceleration.add(0.0);
        }

        double maxOriginalUsage = 0.0;

        for (int i = 1; i <= samples; i++) {
            double tNow = i * dtOriginal;
            double tPrevious = tNow - EPSILON;
            double tPreviousPrevious = tNow - EPSILON - EPSILON;

            for (int k = 0; k < dimensions; k++) {
                double valueNow = fOriginal.get(k).applyAsDouble(tNow);
                double valuePrevious = fOriginal.get(k).applyAsDouble(tPrevious);
                double valuePreviousPrevious = fOriginal.get(k).applyAsDouble(tPreviousPrevious);

                state.set(k, valueNow);
                velocity.set(k, (valueNow - valuePrevious) / EPSILON);
                if (considerAcceleration)
                    acceleration.set(k, (valueNow - 2 * valuePrevious + valuePreviousPrevious) / (EPSILON * EPSILON));
            }

            double currentUsage = usageRatio.applyAsDouble(new Triple<>(state, velocity, acceleration));

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


    public static ToDoubleFunction<Triple<List<Double>, List<Double>, List<Double>>> createHolonomicUsage(
            double maxVelX, double maxVelY, double maxVelTheta,
            double maxAccX, double maxAccY, double maxAccTheta) {
        return pair -> {
            List<Double> s = pair.first;   // State
            List<Double> d = pair.second;  // Derivative
            List<Double> dd = pair.third;  // Second Derivative

            double vxField = d.get(0);
            double vyField = d.get(1);
            double vTheta = Math.abs(d.get(2));
            double heading = s.get(2);

            // Rotate Field-Centric velocities to Robot-Centric
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);
            double vxRobot = vxField * cosH + vyField * sinH;
            double vyRobot = -vxField * sinH + vyField * cosH;

            // Find the bottleneck (velocity)
            double ratioX = Math.abs(vxRobot) / maxVelX;
            double ratioY = Math.abs(vyRobot) / maxVelY;
            double ratioTheta = vTheta / maxVelTheta;
            double velUsage = Math.max(Math.max(ratioX, ratioY), ratioTheta);

            double axField = dd.get(0);
            double ayField = dd.get(1);
            double aTheta = Math.abs(dd.get(2));

            // Rotate Field-Centric accelerations to Robot-Centric
            double axRobot = axField * cosH + ayField * sinH;
            double ayRobot = -axField * sinH + ayField * cosH;

            // Find the bottleneck (acceleration)
            double ratioAccX = Math.abs(axRobot) / maxAccX;
            double ratioAccY = Math.abs(ayRobot) / maxAccY;
            double ratioAccTheta = aTheta / maxAccTheta;
            double accUsage = Math.max(Math.max(ratioAccX, ratioAccY), ratioAccTheta);

            return Math.max(velUsage, Math.sqrt(accUsage)); // Root because acceleration, assume fraction=1
        };
    }

    public Triple<Path, Path, Double> build() {
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

        return new Triple<>(new Path(paths, t -> t >= clock), new Path(holdPaths, t -> false), clock);
    }

    /**
     * Evaluates an arbitrary-degree Bezier curve at parameter {@code u} in [0, 1] using
     * de Casteljau's algorithm, which is numerically stable for all degrees.
     */
    private double deCasteljau(List<Double> points, double u) {
        int n = points.size();
        double[] work = new double[n];
        for (int j = 0; j < n; j++) {
            work[j] = points.get(j);
        }

        for (int r = 1; r < n; r++) {
            for (int j = 0; j < n - r; j++) {
                work[j] = (1.0 - u) * work[j] + u * work[j + 1];
            }
        }

        return work[0];
    }

    private double quinticSmoothstep(double t) {
        if (t <= 0) return 0.0;
        if (t >= 1) return 1.0;
        // 6t^5 - 15t^4 + 10t^3
        return 6 * t * t * t * t * t - 15 * t * t * t * t + 10 * t * t * t;
    }

    /// 2nd order
    private double backwardTaylor(DoubleUnaryOperator f, double c, double t) {
        double fc = f.applyAsDouble(c);
        double fc1 = f.applyAsDouble(c - EPSILON);
        double fc2 = f.applyAsDouble(c - 2 * EPSILON);

        double fp = (3 * fc - 4 * fc1 + fc2) / (2 * EPSILON);
        double fpp = (fc - 2 * fc1 + fc2) / (EPSILON * EPSILON);

        double dx = t - c;
        return fc + fp * dx + fpp * dx * dx / 2;
    }

    ///  2nd order
    private double forwardTaylor(DoubleUnaryOperator f, double c, double t) {
        double fc = f.applyAsDouble(c);
        double fc1 = f.applyAsDouble(c + EPSILON);
        double fc2 = f.applyAsDouble(c + 2 * EPSILON);

        // Forward finite difference coefficients
        double fp = (-3 * fc + 4 * fc1 - fc2) / (2 * EPSILON);
        double fpp = (fc - 2 * fc1 + fc2) / (EPSILON * EPSILON);

        double dx = t - c;
        return fc + fp * dx + fpp * dx * dx / 2;
    }
}