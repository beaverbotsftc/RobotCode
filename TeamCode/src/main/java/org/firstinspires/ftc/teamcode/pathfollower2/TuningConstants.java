package org.firstinspires.ftc.teamcode.pathfollower2;

import java.util.HashMap;

public class TuningConstants {
    public static final HashMap<DOFs.DOF, Double> v = new HashMap<DOFs.DOF, Double>() {{
        put(DOFs.DOF.X, 0.0224959341);
        put(DOFs.DOF.Y, 0.0390850130);
        put(DOFs.DOF.THETA, 0.0200581317);
    }};

    public static final HashMap<DOFs.DOF, Double> a = new HashMap<DOFs.DOF, Double>() {{
        put(DOFs.DOF.X, 0.01);
        put(DOFs.DOF.Y, 0.01);
        put(DOFs.DOF.THETA, 0.01);
    }};

    public static final HashMap<DOFs.DOF, PID.K> pid = new HashMap<DOFs.DOF, PID.K>() {{
        put(DOFs.DOF.X, new PID.K(1, 0, 0));
        put(DOFs.DOF.Y, new PID.K(1, 0, 0));
        put(DOFs.DOF.THETA, new PID.K(1, 0, 0));
    }};

    public static final double[] weights = { 1.0, 0.15, 1.0, 1.0, 1.0, 1.0, 0.35 };
    public static final boolean[] frozenWeights = { false, false, false, false, false, false, true };
}
