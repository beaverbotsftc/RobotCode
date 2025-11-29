package org.beaverbots.beavertracking;

import java.util.Arrays;
import java.util.List;

// https://wiki.purduesigbots.com/software/control-algorithms/ramsete
/// (Checked!) The path dimensionality must be exactly 3, corresponding to x, y, and theta.
/// (Unchecked!) For all points on the path, theta must be exactly tangent to the path (either forwards or backwards).
/// (Unchecked!) The input theta must be in (-inf, inf), and must not suddenly jump from -179.999 to 180.0 or something.
/// The output dimensionality is exactly 2, corresponding to v, omega.
public class RamsetePathTracker implements PathTracker {
    private final Path path;

    private final K k;

    private double time = 0;

    public static final class K {
        public final double b;
        public final double zeta;

        public K(double b, double zeta) {
            this.b = b;
            this.zeta = zeta;
        }
    }

    private static final class HolonomicPose {
        public double x;
        public double y;
        public double theta;

        public HolonomicPose(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }

        public HolonomicPose(List<Double> position) {
            x = position.get(0);
            y = position.get(1);
            theta = position.get(2);
        }
    }

    ///  Only represents velocity as it is non-holonomic
    private static final class NonHolonomicPose {
        public double v;
        public double omega;

        public NonHolonomicPose(double v, double omega) {
            this.v = v;
            this.omega = omega;
        }

        public NonHolonomicPose(List<Double> position) {
            v = position.get(0);
            omega = position.get(1);
        }

        public NonHolonomicPose(HolonomicPose pose) {
            v = Math.sqrt(pose.x * pose.x + pose.y * pose.y);
            omega = pose.theta;
        }
    }

    public RamsetePathTracker(Path path, K k) {
        validatePath(path);
        this.path = path;
        this.k = k;
    }

    public List<Double> update(List<Double> positionList, double dt) {
        if (dt <= 0) throw new IllegalArgumentException("dt must be positive (and non 0)");

        HolonomicPose position = new HolonomicPose(positionList);
        HolonomicPose desiredPosition = new HolonomicPose(path.position(time));
        HolonomicPose holonomicDesiredVelocity = new HolonomicPose(path.velocity(time));
        NonHolonomicPose nonHolonomicDesiredVelocity = new NonHolonomicPose(holonomicDesiredVelocity);

        HolonomicPose globalError = new HolonomicPose(desiredPosition.x - position.x, desiredPosition.y - position.y, desiredPosition.theta - position.theta);
        HolonomicPose localError = globalToLocal(position.theta, globalError);

        double k = 2 * this.k.zeta * Math.sqrt(
                nonHolonomicDesiredVelocity.omega * nonHolonomicDesiredVelocity.omega +
                        this.k.b * nonHolonomicDesiredVelocity.v * nonHolonomicDesiredVelocity.v
        );

        NonHolonomicPose movement = new NonHolonomicPose(
                nonHolonomicDesiredVelocity.v * Math.cos(localError.theta) + k * localError.x,
                nonHolonomicDesiredVelocity.omega + k * localError.theta +
                        this.k.b * nonHolonomicDesiredVelocity.v * localError.y * sinc(localError.theta)
        );

        time += dt;

        return Arrays.asList(movement.v, movement.omega);
    }

    public boolean isFinished() {
        return path.isFinished(time);
    }

    private void validatePath(Path path) {
        if (path.dimensions() != 3) throw new IllegalArgumentException("Path has too many dimensions");
    }

    private HolonomicPose globalToLocal(double theta, HolonomicPose position) {
        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);

        return new HolonomicPose(
                cosTheta * position.x + sinTheta * position.y,
                -sinTheta * position.x + cosTheta * position.y,
                position.theta
        );
    }

    private double sinc(double x) {
        return Math.abs(x) < 1e-6 ?
                1 - x * x / 6 : // Second order taylor expansion, more than good enough for these scales
                Math.sin(x) / x;
    }
}
