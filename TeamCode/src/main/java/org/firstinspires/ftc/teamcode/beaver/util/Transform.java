package org.firstinspires.ftc.teamcode.beaver.util;

import androidx.annotation.NonNull;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

public final class Transform {

    public final double x;
    public final double y;
    public final double theta; // Radians (unnormalized / continuous by default)

    public static final double DEFAULT_EPSILON = 1e-6;

    private static final double INV_SQRT_2 = 1. / Math.sqrt(2);

    // --- Directional & Rotational Unit Constants ---
    public static final Transform ZERO               = new Transform(0, 0, 0);
    public static final Transform FORWARD            = new Transform(1, 0, 0);
    public static final Transform BACKWARD           = new Transform(-1, 0, 0);
    public static final Transform LEFT               = new Transform(0, 1, 0);
    public static final Transform RIGHT              = new Transform(0, -1, 0);
    public static final Transform COUNTERCLOCKWISE   = new Transform(0, 0, 1);
    public static final Transform CLOCKWISE          = new Transform(0, 0, -1);

    public static final Transform FORWARD_LEFT       = new Transform(INV_SQRT_2, INV_SQRT_2, 0);
    public static final Transform FORWARD_RIGHT      = new Transform(INV_SQRT_2, -INV_SQRT_2, 0);
    public static final Transform BACKWARD_LEFT      = new Transform(-INV_SQRT_2, INV_SQRT_2, 0);
    public static final Transform BACKWARD_RIGHT     = new Transform(-INV_SQRT_2, -INV_SQRT_2, 0);

    // =========================================================================
    // Constructors & Factories
    // =========================================================================

    public Transform(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Transform(double x, double y) {
        this(x, y, 0.0);
    }

    public Transform() {
        this(0.0, 0.0, 0.0);
    }

    public Transform(Pose2D pose) {
        this(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.RADIANS));
    }

    public Transform(Pose2D pose, double theta) {
        this(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), theta);
    }

    public Transform(double[] state) {
        if (state == null || state.length < 2 || state.length > 3) {
            throw new IllegalArgumentException(
                    "State array must have length 2 or 3, but got: " + (state == null ? "null" : state.length)
            );
        }
        this.x = state[0];
        this.y = state[1];
        this.theta = (state.length == 3) ? state[2] : 0.0;
    }

    public Transform(List<Double> state) {
        if (state == null || state.size() < 2 || state.size() > 3) {
            throw new IllegalArgumentException(
                    "State list must have size 2 or 3, but got: " + (state == null ? "null" : state.size())
            );
        }
        this.x = state.get(0);
        this.y = state.get(1);
        this.theta = (state.size() == 3) ? state.get(2) : 0.0;
    }

    public Transform(RealVector state) {
        this(state.toArray());
    }

    public static Transform fromPolar(double r, double theta) {
        return new Transform(r * Math.cos(theta), r * Math.sin(theta), 0.0);
    }

    public static Transform fromPolar(double r, double polarAngle, double heading) {
        return new Transform(r * Math.cos(polarAngle), r * Math.sin(polarAngle), heading);
    }

    public static Transform fromDegrees(double x, double y, double degrees) {
        return new Transform(x, y, Math.toRadians(degrees));
    }

    // =========================================================================
    // Accessors & Conversions
    // =========================================================================

    public double getX() { return x; }
    public double getY() { return y; }
    public double getTheta() { return theta; }

    public double getX(DistanceUnit unit) { return unit.fromInches(x); }
    public double getY(DistanceUnit unit) { return unit.fromInches(y); }
    public double getTheta(AngleUnit unit) { return unit.fromRadians(theta); }

    public double heading() {
        return Math.atan2(y, x);
    }

    public Pose2D toPose2d() {
        return toPose2D();
    }

    public Pose2D toPose2D() {
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, theta);
    }

    public Pose2D toPose2D(DistanceUnit distanceUnit, AngleUnit angleUnit) {
        return new Pose2D(distanceUnit, getX(distanceUnit), getY(distanceUnit), angleUnit, getTheta(angleUnit));
    }

    public double[] toArray() {
        return new double[]{x, y, theta};
    }

    public double[] toLateralArray() {
        return new double[]{x, y};
    }

    public List<Double> toList() {
        return Arrays.asList(x, y, theta);
    }

    public RealVector toVector() {
        return new ArrayRealVector(toArray());
    }

    // =========================================================================
    // Fluent Modifiers & Negations
    // =========================================================================

    public Transform withX(double newX) { return new Transform(newX, y, theta); }
    public Transform withY(double newY) { return new Transform(x, newY, theta); }
    public Transform withTheta(double newTheta) { return new Transform(x, y, newTheta); }
    public Transform withLateral(double newX, double newY) { return new Transform(newX, newY, theta); }

    public Transform lateral() { return new Transform(x, y, 0); }
    public Transform angular() { return new Transform(0, 0, theta); }

    /** Negates x, y, and theta */
    public Transform negate() { return new Transform(-x, -y, -theta); }

    /** Negates only x */
    public Transform negateX() { return new Transform(-x, y, theta); }

    /** Negates only y */
    public Transform negateY() { return new Transform(x, -y, theta); }

    /** Negates only theta */
    public Transform negateTheta() { return new Transform(x, y, -theta); }

    /** Negates only x and y (leaves theta unchanged) */
    public Transform negateLateral() { return new Transform(-x, -y, theta); }

    // =========================================================================
    // Vector & Point Math
    // =========================================================================

    public Transform add(Transform other) {
        return new Transform(x + other.x, y + other.y, theta + other.theta);
    }

    public Transform add(double dx, double dy) {
        return new Transform(x + dx, y + dy, theta);
    }

    /** Increments theta by an angle without changing position */
    public Transform turn(double angle) {
        return new Transform(x, y, theta + angle);
    }

    public Transform subtract(Transform other) {
        return new Transform(x - other.x, y - other.y, theta - other.theta);
    }

    /** Scales all components: x, y, and theta */
    public Transform scale(double scalar) {
        return new Transform(x * scalar, y * scalar, theta * scalar);
    }

    /** Scales only lateral components: x and y */
    public Transform scaleLateral(double scalar) {
        return new Transform(x * scalar, y * scalar, theta);
    }

    public Transform multiply(Transform other) {
        return new Transform(x * other.x, y * other.y, theta * other.theta);
    }

    public Transform divide(double scalar) {
        return scale(1.0 / scalar);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public double normSq() {
        return x * x + y * y;
    }

    public double norm(double angularWeight) {
        return Math.sqrt(x * x + y * y + Math.pow(angularWeight * theta, 2));
    }

    public double normSq(double angularWeight) {
        return x * x + y * y + theta * theta * angularWeight * angularWeight;
    }

    public Transform unit() {
        double n = norm();
        if (n < 1e-9) return new Transform(0, 0, theta);
        return new Transform(x / n, y / n, theta);
    }

    public Transform unit(double angularWeight) {
        double n = norm(angularWeight);
        if (n < 1e-9) return new Transform(0, 0, 0);
        return new Transform(x / n, y / n, theta / n);
    }

    public double dot(Transform other) {
        return x * other.x + y * other.y;
    }

    /** 2D Cross product (wedge product): $x_1 y_2 - y_1 x_2$ */
    public double cross(Transform other) {
        return x * other.y - y * other.x;
    }

    /** Perpendicular vector (-y, x) rotated 90 degrees CCW */
    public Transform perpendicular() {
        return new Transform(-y, x, theta);
    }

    public Transform project(Transform onto) {
        double d = onto.normSq();
        if (d < 1e-9) return new Transform(0, 0, theta);
        return onto.scaleLateral(this.dot(onto) / d);
    }

    public double distance(Transform other) {
        return Math.hypot(x - other.x, y - other.y);
    }

    public double distanceSq(Transform other) {
        double dx = x - other.x;
        double dy = y - other.y;
        return dx * dx + dy * dy;
    }

    /** Unnormalized angular difference */
    public double angularDistance(Transform other) {
        return Math.abs(theta - other.theta);
    }

    public double angleTo(Transform other) {
        return Math.atan2(other.y - this.y, other.x - this.x);
    }

    public double relativeAngleTo(Transform other) {
        return angleTo(other) - this.theta;
    }

    /** Linear interpolation without angle wrapping (continuous multi-turn interpolation) */
    public Transform lerp(Transform other, double t) {
        return new Transform(
                x + t * (other.x - x),
                y + t * (other.y - y),
                theta + t * (other.theta - theta)
        );
    }

    // =========================================================================
    // SE(2) Rigid Body & Pose Transformations (Unnormalized Angles)
    // =========================================================================

    /** Rotates position vector and increments heading */
    public Transform rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Transform(
                x * cos - y * sin,
                x * sin + y * cos,
                theta + angle
        );
    }

    /** Rotates position vector without changing heading */
    public Transform rotateLateral(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Transform(
                x * cos - y * sin,
                x * sin + y * cos,
                theta
        );
    }

    public Transform rotateAround(Transform pivot, double angle) {
        return this.subtract(pivot).rotateLateral(angle).add(pivot);
    }

    /** Pose composition: $T_{this} \oplus T_{relative}$ */
    public Transform compose(Transform relative) {
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        return new Transform(
                x + (relative.x * cos - relative.y * sin),
                y + (relative.x * sin + relative.y * cos),
                theta + relative.theta
        );
    }

    /** Relative pose expressing this in frame of origin: $T_{origin}^{-1} \oplus T_{this}$ */
    public Transform relativeTo(Transform origin) {
        double dx = this.x - origin.x;
        double dy = this.y - origin.y;
        double cos = Math.cos(origin.theta);
        double sin = Math.sin(origin.theta);
        return new Transform(
                dx * cos + dy * sin,
                -dx * sin + dy * cos,
                this.theta - origin.theta
        );
    }

    /** Inverse SE(2) transformation */
    public Transform inverse() {
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        return new Transform(
                -x * cos - y * sin,
                x * sin - y * cos,
                -theta
        );
    }

    // =========================================================================
    // se(2) Twist Kinematics (Unnormalized Angles)
    // =========================================================================

    /** Exponential map: twist (vx, vy, omega) to relative pose change */
    public Transform exp() {
        if (Math.abs(theta) < 1e-7) {
            return new Transform(x, y, theta);
        }
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        double dx = (x * sin - y * (1.0 - cos)) / theta;
        double dy = (x * (1.0 - cos) + y * sin) / theta;
        return new Transform(dx, dy, theta);
    }

    /** Logarithmic map: relative pose change to constant-velocity twist */
    public Transform log() {
        if (Math.abs(theta) < 1e-7) {
            return new Transform(x, y, theta);
        }
        double halfTheta = theta / 2.0;
        double cotHalf = 1.0 / Math.tan(halfTheta);
        double vx = halfTheta * (x * cotHalf + y);
        double vy = halfTheta * (-x + y * cotHalf);
        return new Transform(vx, vy, theta);
    }

    public Transform integrateTwist(Transform twist, double dt) {
        return this.compose(twist.scale(dt).exp());
    }

    public Transform toLocalVelocity(Transform robotPose) {
        return this.rotateLateral(-robotPose.theta);
    }

    public Transform toGlobalVelocity(Transform robotPose) {
        return this.rotateLateral(robotPose.theta);
    }

    // =========================================================================
    // Angle Normalization Utilities (Opt-in Only)
    // =========================================================================

    /** Normalizes an angle into $[-\pi, \pi)$ */
    public static double normalizeAngle(double radians) {
        double angle = radians % (2 * Math.PI);
        if (angle > Math.PI) angle -= 2 * Math.PI;
        if (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /** Explicitly returns a new Transform with heading wrapped to $[-\pi, \pi)$ */
    public Transform normalizeHeading() {
        return new Transform(x, y, normalizeAngle(theta));
    }

    // =========================================================================
    // Comparison & Object Methods
    // =========================================================================

    public boolean approxEquals(Transform other) {
        return approxEquals(other, DEFAULT_EPSILON);
    }

    public boolean approxEquals(Transform other, double tolerance) {
        return approxEquals(other, tolerance, tolerance);
    }

    public boolean approxEquals(Transform other, double lateralTolerance, double angularTolerance) {
        return distance(other) <= lateralTolerance && angularDistance(other) <= angularTolerance;
    }

    public boolean equals(Transform other, double lateralTolerance, double angularTolerance) {
        return approxEquals(other, lateralTolerance, angularTolerance);
    }

    public Transform transform(DoubleUnaryOperator[] f) {
        return new Transform(f[0].applyAsDouble(x), f[1].applyAsDouble(y), f[2].applyAsDouble(theta));
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Transform)) return false;
        Transform transform = (Transform) o;
        return Double.compare(transform.x, x) == 0 &&
                Double.compare(transform.y, y) == 0 &&
                Double.compare(transform.theta, theta) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, theta);
    }

    @NonNull
    @Override
    public String toString() {
        return "Transform(x=" + (Math.round(x * 100.0) / 100.0)
                + ", y=" + (Math.round(y * 100.0) / 100.0)
                + ", theta=" + (Math.round(theta * 100.0) / 100.0) + " rad / "
                + (Math.round(Math.toDegrees(theta) * 10.0) / 10.0) + " deg)";
    }
}
