package org.beaverbots.beaver.pathing;

import android.util.Pair;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;

public class PathBuilder {
    private static final double EPSILON = 1e-3;

    protected double clock;
    protected List<DoubleUnaryOperator> f = new ArrayList<>();

    public PathBuilder(List<Double> startingPosition) {
        for (int i = 0; i < startingPosition.size(); i++) {
            final double captured = startingPosition.get(i);
            f.add(t -> captured);
        }
    }

    public PathBuilder(Path path) {
        this(path.position(0));
    }

    public PathBuilder waitFor(double time) {
        clock += time;
        return this;
    }

    private void append(int i, DoubleUnaryOperator fX, double time) {
        final DoubleUnaryOperator fPrevious = f.get(i);
        final double clockCaptured = clock;
        f.set(i, t -> t > clockCaptured ? fX.applyAsDouble(t - clockCaptured) : fPrevious.applyAsDouble(t));
    }

    public PathBuilder stop(double easingTime, double time) {
        for (int i = 0; i < f.size(); i++) {
            final double target = f.get(i).applyAsDouble(clock);
            appendEase(i, t -> target, easingTime);
        }
        clock += time;

        return this;
    }

    public PathBuilder stop(double time) {
        return this.stop(0, time);
    }

    public PathBuilder moveTo(List<Double> x, double time) {
        for (int i = 0; i < x.size(); i++) {
            final double target = x.get(i);
            appendEase(i, t -> target, time);
        }

        clock += time;

        return this;
    }

    private void appendEase(int i, DoubleUnaryOperator fNew, double easingTime) {
        final DoubleUnaryOperator fPrevious = f.get(i);
        final double clockCaptured = clock;

        // Wrap fNew so it receives local time (t - clockCaptured)
        // easeTransition still manages the blend in global time t
        f.set(i, easeTransition(fPrevious, t -> fNew.applyAsDouble(t - clockCaptured), clockCaptured, easingTime));
    }


    public PathBuilder linearTo(List<Double> x, double easingTime, double time) {
        for (int i = 0; i < x.size(); i++) {
            final DoubleUnaryOperator fPrevious = f.get(i);
            final double target = x.get(i);

            final double start = fPrevious.applyAsDouble(clock);

            DoubleUnaryOperator fNew = t -> {
                if (t <= 0) return start;

                double u = t / time;

                return start + u * (target - start);
            };

            appendEase(i, fNew, easingTime);
        }

        clock += time;

        return this;
    }

    public PathBuilder linearTo(List<Double> x, double time) {
        return this.linearTo(x, 0, time);
    }

    public PathBuilder bezierTo(List<Double> control1, List<Double> control2, List<Double> target, double easingTime, double time) {
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

            appendEase(i, fNew, easingTime);
        }

        clock += time;

        return this;
    }

    public PathBuilder bezierTo(List<Double> control1, List<Double> control2, List<Double> target, double time) {
        return this.bezierTo(control1, control2, target, 0, time);
    }

    public PathBuilder transform(List<DoubleUnaryOperator> transformers) {
        for (int i = 0; i < f.size(); i++) {
            final DoubleUnaryOperator original = f.get(i);
            final DoubleUnaryOperator transform = transformers.get(i);

            f.set(i, t -> transform.applyAsDouble(original.applyAsDouble(t)));
        }

        return this;
    }

    private DoubleUnaryOperator easeTransition(
            DoubleUnaryOperator f0,
            DoubleUnaryOperator f1,
            double c,
            double easingTime
    ) {
        final double b = c + easingTime;

        return t -> {
            if (t <= c) {
                return f0.applyAsDouble(t);
            }

            if (t < b) {
                // Blend parameter u in [0,1]
                double u = (t - c) / easingTime;
                double s = quinticSmoothstep(u);

                // Smooth continuation from the left
                double yTaylor = taylor(f0, c, t);

                // Future (target) path
                double y1 = f1.applyAsDouble(t);

                // Blend them
                return (1 - s) * yTaylor + s * y1;
            }

            // After easing window: follow f1
            return f1.applyAsDouble(t);
        };
    }

    ///  First is the actual path, the second is the path you can use to hold the end position
    public Pair<Path, Path> build() {
        List<PathAxis> paths = new ArrayList<>();
        List<PathAxis> holdPaths = new ArrayList<>();
        for (int i = 0; i < f.size(); i++) {
            paths.add(new PathAxis(f.get(i), 0, clock));
            final double endpoint = f.get(i).applyAsDouble(clock);
            holdPaths.add(new PathAxis(t -> endpoint, 0, Double.POSITIVE_INFINITY));
        }

        return new Pair<>(new Path(paths, t -> t >= clock), new Path(holdPaths, t -> false));
    }

    private double quinticSmoothstep(double t) {
        if (t <= 0) return 0.0;
        if (t >= 1) return 1.0;

        // 6t^5 − 15t^4 + 10t^3
        return t * t * t * (t * (6 * t - 15) + 10);
    }


    /// 2nd order
    private double taylor(DoubleUnaryOperator f, double c, double t) {
        double fc = f.applyAsDouble(c);
        double fc1 = f.applyAsDouble(c - EPSILON);
        double fc2 = f.applyAsDouble(c - 2 * EPSILON);

        double fp = (3 * fc - 4 * fc1 + fc2) / (2 * EPSILON);

        double fpp = (fc - 2 * fc1 + fc2) / (EPSILON * EPSILON);

        double dx = t - c;
        return fc + fp * dx + 0.5 * fpp * dx * dx;
    }
}