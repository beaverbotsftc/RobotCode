package org.firstinspires.ftc.teamcode;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;

public class CrossModeStorage {
    public static DrivetrainState position = new DrivetrainState(0, 0, 0);
    public static RealMatrix covariance = new Array2DRowRealMatrix(new double[][] {{144, 0, 0}, {0, 144, 0}, {0, 0, 3}});
    public static Side side = Side.RED;
}
