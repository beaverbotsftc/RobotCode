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

    // Desired base RPM (we'll adjust angle instead of speed)
    public static final double BASE_RPM = 4500.0;

    // Shooter wheel radius (m)
    public static final double WHEEL_RADIUS_M = 0.05; // 5 cm

    // Friction coefficient (0–1): 1 = no loss, 0.85 = 15% velocity loss
    public static final double FRICTION_COEFF = 0.9;

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
     * Computes shooter wheel angle for a given horizontal distance (m)
     * using a fixed RPM (launch speed). Accounts for friction losses.
     *
     * @param distanceM  horizontal distance to target (m)
     * @return ShotParameters containing RPM, angle, and message
     */
    public static ShotParameters computeShotForDistanceMeters(double distanceM) {
        double h0 = SHOOTER_EXIT_HEIGHT_M;
        double hT = TARGET_HEIGHT_M;

        // Convert base RPM to effective velocity (after friction loss)
        double vWheel = BASE_RPM / 60.0 * 2.0 * Math.PI * WHEEL_RADIUS_M;
        double v0 = vWheel * FRICTION_COEFF;

        // Compute the required launch angle (in radians)
        double g = G;
        double a = (g * distanceM * distanceM) / (2 * v0 * v0);
        double b = distanceM;
        double c = h0 - hT;

        // Solve quadratic form for tan(theta)
        // hT - h0 = x*tan(theta) - (g*x^2)/(2*v0^2*cos^2(theta))
        // Rearranged into t = tan(theta):  a*t^2 - b*t + c + a = 0
        double A = a;
        double B = -b;
        double C = c + a;

        double discriminant = B * B - 4 * A * C;
        if (discriminant < 0) {
            return new ShotParameters(BASE_RPM, 45.0, false, "No valid angle solution.");
        }

        // Two possible angles (lower and higher arcs)
        double t1 = (-B + Math.sqrt(discriminant)) / (2 * A);
        double t2 = (-B - Math.sqrt(discriminant)) / (2 * A);

        // Choose the higher arc (steeper shot)
        double chosenTan = Math.max(t1, t2);
        double thetaDeg = Math.toDegrees(Math.atan(chosenTan));

        // Feasibility check
        if (thetaDeg < 10 || thetaDeg > 75) {
            return new ShotParameters(BASE_RPM, thetaDeg, false,
                    String.format("Angle %.1f° out of range", thetaDeg));
        }

        return new ShotParameters(BASE_RPM, thetaDeg, true, "OK");
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
