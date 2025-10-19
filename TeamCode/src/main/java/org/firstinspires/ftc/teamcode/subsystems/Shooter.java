package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Shooter math utility and subsystem.
 *
 * Can be called from anywhere (TeleOp, Auto, or another subsystem) to get
 * shooter wheel RPM and shooter angle for a given target distance.
 *
 * Example:
 *     Shooter.ShotParameters shot = Shooter.computeShotForDistanceMeters(3.0);
 *     double rpm = shot.motorRpm;
 *     double angle = shot.angleDeg;
 */
public class Shooter {

    // ---------------- Constants (tune these) ----------------
    // Target height = 40 inches (user request)
    public static final double TARGET_HEIGHT_IN = 40.0;
    public static final double INCH_TO_M = 0.0254;
    public static final double TARGET_HEIGHT_M = TARGET_HEIGHT_IN * INCH_TO_M;

    // Shooter exit height — adjust for your robot
    public static final double SHOOTER_EXIT_HEIGHT_IN = 8.0;
    public static final double SHOOTER_EXIT_HEIGHT_M = SHOOTER_EXIT_HEIGHT_IN * INCH_TO_M;

    // Fixed arc angle so the ball “arches” onto the target
    public static final double ARC_ANGLE_DEG = 50.0;

    // Shooter wheel radius (m)
    public static final double WHEEL_RADIUS_M = 0.05; // 5 cm

    // Motor limits
    public static final double MAX_RPM = 6000.0;
    public static final double MIN_RPM = 200.0;

    // Gravity
    public static final double G = 9.80665;

    // ---------------- Return structure ----------------
    public static class ShotParameters {
        public final double motorRpm;
        public final double angleDeg;
        public final boolean feasible;
        public final String message;

        public ShotParameters(double motorRpm, double angleDeg, boolean feasible, String message) {
            this.motorRpm = motorRpm;
            this.angleDeg = angleDeg;
            this.feasible = feasible;
            this.message = message;
        }
    }

    // ---------------- Core logic ----------------

    /**
     * Computes shooter wheel RPM and angle for a given horizontal distance (in meters).
     * Uses a fixed “arched” shot angle to ensure the ball travels upward and lands on target.
     *
     * @param distanceM  horizontal distance to target in meters
     * @return ShotParameters containing motor RPM, angle, and status message
     */
    public static ShotParameters computeShotForDistanceMeters(double distanceM) {
        double thetaDeg = ARC_ANGLE_DEG;
        double theta = Math.toRadians(thetaDeg);
        double h0 = SHOOTER_EXIT_HEIGHT_M;
        double hT = TARGET_HEIGHT_M;

        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);
        double denom = 2.0 * cosTheta * cosTheta * (h0 + distanceM * tanTheta - hT);

        if (denom <= 0) {
            String msg = "Unfeasible shot: geometry invalid for chosen angle. Increase angle or shooter height.";
            return new ShotParameters(0.0, thetaDeg, false, msg);
        }

        double v0sq = G * distanceM * distanceM / denom;
        if (v0sq <= 0) {
            return new ShotParameters(0.0, thetaDeg, false, "No valid solution (negative velocity squared).");
        }

        double v0 = Math.sqrt(v0sq);
        double wheelRpm = v0 / (2.0 * Math.PI * WHEEL_RADIUS_M) * 60.0;

        boolean feasible = true;
        String msg = "OK";

        if (wheelRpm > MAX_RPM) {
            msg = String.format("RPM capped at %.0f (required %.0f)", MAX_RPM, wheelRpm);
            wheelRpm = MAX_RPM;
        } else if (wheelRpm < MIN_RPM) {
            msg = String.format("RPM raised to %.0f (required %.0f)", MIN_RPM, wheelRpm);
            wheelRpm = MIN_RPM;
        }

        return new ShotParameters(wheelRpm, thetaDeg, feasible, msg);
    }

    /**
     * Convenience overload for distance in inches.
     *
     * @param distanceIn  horizontal distance to target in inches
     * @return ShotParameters
     */
    public static ShotParameters computeShotForDistanceInches(double distanceIn) {
        return computeShotForDistanceMeters(distanceIn * INCH_TO_M);
    }
}
