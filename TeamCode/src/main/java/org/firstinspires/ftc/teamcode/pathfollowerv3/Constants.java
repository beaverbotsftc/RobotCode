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

        PidXProportional,
        PidXIntegral,
        PidXDerivative,
        PidYProportional,
        PidYIntegral,
        PidYDerivative,
        PidThetaProportional,
        PidThetaIntegral,
        PidThetaDerivative,

        PathFollowerXVelocityGain,
        PathFollowerXAccelerationGain,
        PathFollowerXDeviationGain,
        PathFollowerXConstant,
        PathFollowerYVelocityGain,
        PathFollowerYAccelerationGain,
        PathFollowerYDeviationGain,
        PathFollowerYConstant,
        PathFollowerThetaVelocityGain,
        PathFollowerThetaAccelerationGain,
        PathFollowerThetaDeviationGain,
        PathFollowerThetaConstant,
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

        put(Constant.PidXProportional, 1.0);
        put(Constant.PidXIntegral, 1.0);
        put(Constant.PidXDerivative, 1.0);
        put(Constant.PidYProportional, 1.0);
        put(Constant.PidYIntegral, 1.0);
        put(Constant.PidYDerivative, 1.0);
        put(Constant.PidThetaProportional, 1.0);
        put(Constant.PidThetaIntegral, 1.0);
        put(Constant.PidThetaDerivative, 1.0);

        put(Constant.PathFollowerXVelocityGain, 0.0224959341);
        put(Constant.PathFollowerXAccelerationGain, 0.1);
        put(Constant.PathFollowerXDeviationGain, 1.0);
        put(Constant.PathFollowerXConstant, 0.0);
        put(Constant.PathFollowerYVelocityGain, 0.0390850130);
        put(Constant.PathFollowerYAccelerationGain, 0.1);
        put(Constant.PathFollowerYDeviationGain, 1.0);
        put(Constant.PathFollowerYConstant, 0.0);
        put(Constant.PathFollowerThetaVelocityGain, 0.0200581317);
        put(Constant.PathFollowerThetaAccelerationGain, 0.1);
        put(Constant.PathFollowerThetaDeviationGain, 1.0);
        put(Constant.PathFollowerThetaConstant, 0.0);
    }};
}
