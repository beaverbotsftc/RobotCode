package org.firstinspires.ftc.teamcode;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.beaverbots.beaver.util.Transform;

public class CrossModeStorage {
    public static Transform position;
    public static RealMatrix covariance;
    public static Side side;

    public static void reset() {
        position = new Transform(0, 0, 0);
        covariance = new Array2DRowRealMatrix(new double[][] {{144, 0, 0}, {0, 144, 0}, {0, 0, 3}});
        side = Side.RED;
    }

    static {
        reset();
    }
}
