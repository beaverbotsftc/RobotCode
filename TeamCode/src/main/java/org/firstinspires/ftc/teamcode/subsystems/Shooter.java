package org.firstinspires.ftc.teamcode.subsystems;

public class Shooter {

    // --- Conversion constants ---
    public static final double INCH_TO_M = 0.0254;

    // --- Target geometry ---
    public static final double TARGET_HEIGHT_IN = 40.0;
    public static final double TARGET_HEIGHT_M = TARGET_HEIGHT_IN * INCH_TO_M;
    public static final double SHOOTER_EXIT_HEIGHT_IN = 8.0;
    public static final double SHOOTER_EXIT_HEIGHT_M = SHOOTER_EXIT_HEIGHT_IN * INCH_TO_M;

    // --- Physics constants ---
    public static final double G = 9.80665;

    // --- Ball physical properties (tune for your setup) ---
    // --- Ball physical properties (5" diameter, 0.165 lbs) ---
    public static double m_b = 0.0748;    // kg
    public static double d_b = 0.127;     // m (5 inches)
    public static double I_b = (2.0 / 5.0) * m_b * Math.pow(d_b / 2.0, 2.0); // kg·m²


    // --- Flywheel physical properties ---
    public static double m_w = 0.2;       // kg
    public static double I_w = 0.0008;    // kg·m²
    public static double d_w = 0.096;     // m

    // --- Hood geometry and friction ---
    public static double w = 0.09;        // hood gap (m)
    public static double mu_w = 0.9;      // friction coefficient
    public static double k = 1e4;         // spring constant (N/m)
    public static double defaultHoodContactLength = 0.12; // m

    // ---------------- Internal helpers for constants ----------------
    private static double calcA() { return d_w / (2.0 * I_w); } // Eq. 22
    private static double calcB() { return 1.0 / (m_b / 2.0 + 2.0 * I_b / (w * w)); } // Eq. 23
    private static double calcC() { return mu_w * k * (d_b - w); } // Eq. 24
    private static double calcD() { return mu_w * m_b / (w + d_w); } // Eq. 25

    // ---------------- ShotParameters data structure ----------------
    public static class ShotParameters {
        public final double motorRpm;
        public final double wheelOmegaRadPerSec;
        public final double ballExitVelocityMps;
        public final double launchAngleDeg;
        public final boolean feasible;
        public final String message;

        public ShotParameters(double motorRpm, double wheelOmegaRadPerSec, double ballExitVelocityMps,
                              double launchAngleDeg, boolean feasible, String message) {
            this.motorRpm = motorRpm;
            this.wheelOmegaRadPerSec = wheelOmegaRadPerSec;
            this.ballExitVelocityMps = ballExitVelocityMps;
            this.launchAngleDeg = launchAngleDeg;
            this.feasible = feasible;
            this.message = message;
        }
    }

    // ---------------- Hughes hood-shooter equations ----------------

    /** Compute vb_x = e^(-B*D*x) * sqrt(C*(e^(2*B*D*x) - 1)/D) */
    public static double computeVb_x(double xMeters) {
        double B = calcB();
        double C = calcC();
        double D = calcD();

        if (D <= 0.0 || C <= 0.0) return 0.0;

        double BDx = B * D * xMeters;
        double exp2BDx = Math.exp(2.0 * BDx);
        double insideSqrt = (C * (exp2BDx - 1.0)) / D;

        if (insideSqrt <= 0.0) return 0.0;
        return Math.exp(-BDx) * Math.sqrt(insideSqrt);
    }

    /** Compute vb_steady = ωw_i * dw/4 + A/B */
    public static double computeVbSteady(double omega_w_i) {
        double A = calcA();
        double B = calcB();
        return omega_w_i * d_w / 4.0 + A / B;
    }

    /** Compute v_b,f = min(vb_x, vb_steady) */
    public static double computeBallExitVelocityFromRpm(double initialRpm, double hoodContactLengthMeters) {
        double omega_w_i = rpmToRadPerSec(initialRpm);
        double vb_x = computeVb_x(hoodContactLengthMeters);
        double vb_steady = computeVbSteady(omega_w_i);
        return Math.min(vb_x, vb_steady);
    }

    // ---------------- Projectile helper equations ----------------

    private static double[] computeAngleSolutions(double v0, double distanceM) {
        double D = distanceM;
        double h0 = SHOOTER_EXIT_HEIGHT_M;
        double hT = TARGET_HEIGHT_M;

        double a = (G * D * D) / (2.0 * v0 * v0);
        double b = D;
        double c = h0 - hT;

        double A = a;
        double B = -b;
        double C = c + a;

        double disc = B * B - 4.0 * A * C;
        if (disc < 0) return null;

        double sqrtDisc = Math.sqrt(disc);
        double t1 = (-B + sqrtDisc) / (2.0 * A);
        double t2 = (-B - sqrtDisc) / (2.0 * A);

        double theta1 = Math.toDegrees(Math.atan(t1));
        double theta2 = Math.toDegrees(Math.atan(t2));

        return new double[]{Math.min(theta1, theta2), Math.max(theta1, theta2)};
    }

    private static double chooseSlightArcAngle(double v0, double distanceM) {
        double[] sols = computeAngleSolutions(v0, distanceM);
        if (sols == null) return Double.NaN;

        double low = sols[0];
        double high = sols[1];
        double chosen = (low > 0.0) ? low : ((high > 0.0) ? high : Double.NaN);

        return (chosen > 55.0) ? Double.NaN : chosen;
    }

    // ---------------- Main API methods ----------------

    public static ShotParameters computeRequiredRpmForAngleMeters(double distanceMeters, double angleDeg, double hoodContactLengthMeters) {
        double theta = Math.toRadians(angleDeg);
        double D = distanceMeters;
        double h0 = SHOOTER_EXIT_HEIGHT_M;
        double hT = TARGET_HEIGHT_M;

        double cos = Math.cos(theta);
        double tan = Math.tan(theta);
        double denom = 2.0 * cos * cos * (h0 + D * tan - hT);
        if (denom <= 0.0)
            return new ShotParameters(0.0, 0.0, 0.0, angleDeg, false, "Invalid geometry for that angle.");

        double v0sq = G * D * D / denom;
        if (v0sq <= 0.0)
            return new ShotParameters(0.0, 0.0, 0.0, angleDeg, false, "Negative v0² (invalid).");

        double v_req = Math.sqrt(v0sq);
        double vb_x = computeVb_x(hoodContactLengthMeters);

        if (vb_x >= v_req) {
            return new ShotParameters(0.0, 0.0, vb_x, angleDeg, true,
                    String.format("Hood-limited speed %.3f >= required %.3f (no extra RPM needed)", vb_x, v_req));
        }

        // invert vb_steady
        double A = calcA();
        double B = calcB();
        double v_offset = A / B;
        double numerator = v_req - v_offset;
        if (numerator <= 0.0)
            return new ShotParameters(0.0, 0.0, vb_x, angleDeg, false, "v_req too low — check constants.");

        double omega_req_rad = numerator * 4.0 / d_w;
        double rpm_req = radPerSecToRpm(omega_req_rad);

        double vb_steady = computeVbSteady(omega_req_rad);
        double vb_final = Math.min(vb_x, vb_steady);

        boolean feasible = vb_final + 1e-9 >= v_req;
        String msg = feasible
                ? String.format("RPM %.1f for %.1f° (v_req=%.3f, vb_x=%.3f, vb_steady=%.3f)", rpm_req, angleDeg, v_req, vb_x, vb_steady)
                : String.format("Impossible: v_req=%.3f, vb_x=%.3f, vb_steady=%.3f", v_req, vb_x, vb_steady);

        return new ShotParameters(rpm_req, omega_req_rad, vb_final, angleDeg, feasible, msg);
    }

    // Convenience wrapper (inches)
    public static ShotParameters computeRequiredRpmForAngleInches(double distanceIn, double angleDeg, double hoodContactLengthIn) {
        return computeRequiredRpmForAngleMeters(distanceIn * INCH_TO_M, angleDeg, hoodContactLengthIn * INCH_TO_M);
    }

    // ---------------- Unit conversion helpers ----------------
    private static double rpmToRadPerSec(double rpm) {
        return rpm * 2.0 * Math.PI / 60.0;
    }

    private static double radPerSecToRpm(double radPerSec) {
        return radPerSec * 60.0 / (2.0 * Math.PI);
    }

    // ---------------- Optional parameter setters ----------------
    public static void setBallParameters(double massKg, double inertiaKgM2, double diameterM) {
        m_b = massKg;
        I_b = inertiaKgM2;
        d_b = diameterM;
    }

    public static void setWheelParameters(double massKg, double inertiaKgM2, double diameterM) {
        m_w = massKg;
        I_w = inertiaKgM2;
        d_w = diameterM;
    }

    public static void setHoodParameters(double hoodGapW, double mu, double kSpring, double contactLen) {
        w = hoodGapW;
        mu_w = mu;
        k = kSpring;
        defaultHoodContactLength = contactLen;
    }
}
