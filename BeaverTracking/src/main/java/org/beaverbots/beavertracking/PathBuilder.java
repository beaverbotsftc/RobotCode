package org.beaverbots.beavertracking;

import android.util.Pair;

import org.beaverbots.BeaverCommand.Command;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;

public class PathBuilder {
    private static final double EPSILON = 1e-3;

    protected double clock;
    protected List<DoubleUnaryOperator> f = new ArrayList<>();
    protected List<Pair<Double, Command>> commands = new ArrayList<>();

    public PathBuilder(List<Double> startingPosition) {
        for (int i = 0; i < startingPosition.size(); i++) {
            final double captured = startingPosition.get(i);
            f.add(t -> captured);
        }
    }

    public PathBuilder wait(double time) {
        clock += time;
        return this;
    }

    public PathBuilder addCommand(Command command) {
        commands.add(new Pair<>(clock, command));
        return this;
    }

    public PathBuilder moveTo(List<Double> x, double time) {
        for (int i = 0; i < x.size(); i++) {
            final DoubleUnaryOperator fPrevious = f.get(i);
            final double target = x.get(i);

            DoubleUnaryOperator fNew = t -> target;

            f.set(i, easeTransition(fPrevious, fNew, clock, time));
        }

        clock += time;

        return this;
    }

    public PathBuilder linearToEase(List<Double> x, double easingTime, double time) {
        for (int i = 0; i < x.size(); i++) {
            final DoubleUnaryOperator fPrevious = f.get(i);
            final double target = x.get(i);
            final double clockCaptured = clock;

            DoubleUnaryOperator fNew = t -> {
                double start = fPrevious.applyAsDouble(clockCaptured);
                if (t <= clockCaptured) return start;

                double u = (t - clockCaptured) / time;

                return start + u * (target - start);
            };

            f.set(i, easeTransition(fPrevious, fNew, clock, easingTime));
        }

        clock += time;

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
