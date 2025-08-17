package org.firstinspires.ftc.teamcode.pathfollowerv3;

import java.util.HashMap;

public final class Constants {
    public enum Constant {
        OdometryOffsetX,
        OdometryOffsetY,

        LateralSpeed,
        RotationalSpeed,

        LeftFrontMotorPower,
        RightFrontMotorPower,
        LeftBackMotorPower,
        RightBackMotorPower,

        MaxDrivetrainMotorPower,

        VelocityGainX,
        VelocityGainY,
        VelocityGainTheta,
        AccelerationGainX,
        AccelerationGainY,
        AccelerationGainTheta,

        PidXProportional,
        PidXIntegral,
        PidXDerivative,
        PidYProportional,
        PidYIntegral,
        PidYDerivative,
        PidThetaProportional,
        PidThetaIntegral,
        PidThetaDerivative,
    }

    public static HashMap<Constant, Double> weights = new HashMap<Constant, Double>() {{
        /*
            Set the odometry pod positions relative to the point that the odometry computer tracks around.
            The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is. Left of the center is a positive number, right of center is a negative number.
            The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is. forward of center is a positive number, backwards is a negative number.
        */
        put(Constant.OdometryOffsetX, -84.3375);
        put(Constant.OdometryOffsetY, 76.0000);

        put(Constant.LateralSpeed, 1.0);
        put(Constant.RotationalSpeed, 0.15);

        put(Constant.LeftFrontMotorPower, 1.0);
        put(Constant.RightFrontMotorPower, 1.0);
        put(Constant.LeftBackMotorPower, 1.0);
        put(Constant.RightBackMotorPower, 1.0);

        put(Constant.MaxDrivetrainMotorPower, 0.35);

        put(Constant.VelocityGainX, 0.0224959341);
        put(Constant.VelocityGainY, 0.0390850130);
        put(Constant.VelocityGainTheta, 0.0200581317);
        put(Constant.AccelerationGainX, 0.01);
        put(Constant.AccelerationGainY, 0.01);
        put(Constant.AccelerationGainTheta, 0.01);

        put(Constant.PidXProportional, 1.0);
        put(Constant.PidXIntegral, 1.0);
        put(Constant.PidXDerivative, 1.0);
        put(Constant.PidYProportional, 1.0);
        put(Constant.PidYIntegral, 1.0);
        put(Constant.PidYDerivative, 1.0);
        put(Constant.PidThetaProportional, 1.0);
        put(Constant.PidThetaIntegral, 1.0);
        put(Constant.PidThetaDerivative, 1.0);
    }};
}
