package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Shooter math utility and subsystem.
 *
 * - Prefers slight (low) arc when possible.
 * - If fixed BASE_RPM would force an impractical steep arc, computeRequiredRpmForAngle(...)
 *   can be used to find what RPM you must set to achieve a chosen slight arc.
 */
public class Shooter {

    // ---------------- Constants ----------------
    public static final double TARGET_HEIGHT_IN = 40.0;      // target height (inches)
    public static final double INCH_TO_M = 0.0254;
    public static final double TARGET_HEIGHT_M = TARGET_HEIGHT_IN * INCH_TO_M;

    public static final double SHOOTER_EXIT_HEIGHT_IN = 8.0; // shooter exit height (inches)
    public static final double SHOOTER_EXIT_HEIGHT_M = SHOOTER_EXIT_HEIGHT_IN * INCH_TO_M;

    // Wheel diameter: 96 mm -> radius 0.048 m
    public static final double WHEEL_DIAMETER_MM = 96.0;
    public static final double WHEEL_RADIUS_M = (WHEEL_DIAMETER_MM / 1000.0) / 2.0;

    // Fixed/base RPM (your nominal speed)
    public static final double BASE_RPM = 4700.0;

    // Friction coefficient (0..1). 1.0 = perfect transfer v_wheel->v0.
    public static final double FRICTION_COEFF = 0.9;

    // Physics
    public static final double G = 9.80665;

    // Derived speeds
    public static final double WHEEL_SPEED_MPS = 2.0 * Math.PI * WHEEL_RADIUS_M * (BASE_RPM / 60.0);
    public static final double LAUNCH_SPEED_MPS = WHEEL_SPEED_MPS * FRICTION_COEFF;

    // ---------------- Return structure ----------------
    public static class ShotParameters {
        public final double motorRpm;   // RPM to set (may be BASE_RPM or computed)
        public final double angleDeg;   // launch angle to set
        public final boolean feasible;
        public final String message;

        public ShotParameters(double motorRpm, double angleDeg, boolean feasible, String message) {
            this.motorRpm = motorRpm;
            this.angleDeg = angleDeg;
            this.feasible = feasible;
            this.message = message;
        }
    }

    // ---------------- Core methods ----------------

    /**
     * Compute both angle solutions (in degrees) for given v0 and distance.
     * Returns double[2] = {thetaLowDeg, thetaHighDeg}. Values may be negative.
     */
    private static double[] computeAngleSolutions(double v0, double distanceM) {
        double D = distanceM;
        double h0 = SHOOTER_EXIT_HEIGHT_M;
        double hT = TARGET_HEIGHT_M;

        // Quadratic in t = tan(theta): a t^2 - b t + (c + a) = 0
        double a = (G * D * D) / (2.0 * v0 * v0);
        double b = D;
        double c = h0 - hT;

        double A = a;
        double B = -b;
        double C = c + a;

        double disc = B * B - 4.0 * A * C;
        if (disc < 0) {
            return null; // no real solutions
        }

        double sqrtDisc = Math.sqrt(disc);
        double t1 = (-B + sqrtDisc) / (2.0 * A);
        double t2 = (-B - sqrtDisc) / (2.0 * A);

        double theta1 = Math.toDegrees(Math.atan(t1));
        double theta2 = Math.toDegrees(Math.atan(t2));

        // ensure theta1 <= theta2 (low, high)
        double low = Math.min(theta1, theta2);
        double high = Math.max(theta1, theta2);
        return new double[]{low, high};
    }

    /**
     * Compute the angle when using BASE_RPM. Prefer the smaller positive angle (slight arc).
     * If only a large arc exists (or no positive solution), returns feasible=false.
     */
    public static ShotParameters computeSlightArcForDistanceMeters(double distanceM) {
        double v0 = LAUNCH_SPEED_MPS;
        double[] sols = computeAngleSolutions(v0, distanceM);
        if (sols == null) {
            return new ShotParameters(BASE_RPM, 0.0, false, "No angle solution at BASE_RPM (out of range).");
        }

        double low = sols[0];
        double high = sols[1];

        // pick the small positive angle if available, else the small-magnitude positive one.
        double chosen = Double.NaN;
        if (low > 0.0) {
            chosen = low;
        } else if (high > 0.0) {
            // low <= 0 < high : both not positive, high is the only positive (often steep)
            chosen = high;
        } else {
            // both <= 0: no forward upward solution
            return new ShotParameters(BASE_RPM, Math.min(low, high), false,
                    "Both solutions non-positive (no forward-upward shot at BASE_RPM).");
        }

        // If the chosen angle is steep (> ~55°) we consider that "large arc" and return infeasible
        if (chosen > 55.0) {
            return new ShotParameters(BASE_RPM, chosen, false,
                    String.format("BASE_RPM produces only a large arc (%.2f°).", chosen));
        }

        return new ShotParameters(BASE_RPM, chosen, true, String.format("Angle chosen: %.2f° (slight arc).", chosen));
    }

    /**
     * Compute required wheel RPM to achieve a chosen launch angle (deg) for given distance (m).
     * Accounts for friction coefficient: wheel_speed = required_v0 / FRICTION_COEFF
     */
    public static ShotParameters computeRequiredRpmForAngleMeters(double distanceM, double angleDeg) {
        double theta = Math.toRadians(angleDeg);
        double D = distanceM;
        double h0 = SHOOTER_EXIT_HEIGHT_M;
        double hT = TARGET_HEIGHT_M;

        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double denom = 2.0 * cos * cos * (h0 + D * tan - hT);
        if (denom <= 0.0) {
            return new ShotParameters(0.0, angleDeg, false, "Geometry invalid for that angle.");
        }

        double v0sq = G * D * D / denom;
        if (v0sq <= 0.0) {
            return new ShotParameters(0.0, angleDeg, false, "Computed negative v0^2 (invalid).");
        }

        double v0 = Math.sqrt(v0sq);
        double wheelSpeed = v0 / FRICTION_COEFF; // wheel tangential speed needed
        double rpm = wheelSpeed / (2.0 * Math.PI * WHEEL_RADIUS_M) * 60.0;

        // sanity: rpm must be positive
        if (rpm <= 0.0) {
            return new ShotParameters(0.0, angleDeg, false, "Computed non-positive RPM.");
        }

        return new ShotParameters(rpm, angleDeg, true, String.format("Required RPM %.1f for %.1f°", rpm, angleDeg));
    }

    /** Convenience: inches -> meters */
    public static ShotParameters computeSlightArcForDistanceInches(double distanceIn) {
        return computeSlightArcForDistanceMeters(distanceIn * INCH_TO_M);
    }

    public static ShotParameters computeRequiredRpmForAngleInches(double distanceIn, double angleDeg) {
        return computeRequiredRpmForAngleMeters(distanceIn * INCH_TO_M, angleDeg);
    }
}
