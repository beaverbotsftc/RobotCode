package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import java.util.List;

public final class Constants {
    public static double drivetrainPowerConversionFactorX = 1 / 62.45693318101283 / (1 - 0.1);
    public static double drivetrainPowerConversionFactorY = 1 / 62.45693318101283 / (1 - 0.3);
    public static double drivetrainPowerConversionFactorTheta = 1 / 6.874573388870918;

    public static double drivetrainFrontLeftFactor = 1;
    public static double drivetrainFrontRightFactor = 1;
    public static double drivetrainBackLeftFactor = 1;
    public static double drivetrainBackRightFactor = 1;


    public static double pidPX = 0.05;
    public static double pidIX = 0;
    public static double pidDX = 0;
    public static double pidFVelocityX = 0;
    public static double pidFAccelerationX = 0;
    public static double pidGammaX = 1;
    public static double pidPY = 0.05;
    public static double pidIY = 0;
    public static double pidDY = 0;
    public static double pidFVelocityY = 0;
    public static double pidFAccelerationY = 0;
    public static double pidGammaY = 300;
    public static double pidPTheta = 1;
    public static double pidITheta = 0;
    public static double pidDTheta = 0;
    public static double pidFVelocityTheta = 0;
    public static double pidFAccelerationTheta = 0;
    public static double pidGammaTheta = 3000;

    public static double pidTau = 0.1;

    // Left of center is positive, right of center is negative
    public static double pinpointXOffset = 108.675; // mm
    // Forward of center is positive, behind center is negative
    public static double pinpointYOffset = 0.4; // mm

    public static double lateralVariancePinpoint = 0.000683634022087;
    public static double angularVariancePinpoint = 0.0000000268388027135 * 1000;

    public static double minLateralVariance = 0;
    public static double minAngularVariance = 0;

    public static double turretLatency = 0.08;
    public static double turretBounds = Math.toRadians(90);
    public static double turretAngularBias = 0;

    public static double turretDelta = 0.001;
    public static double shooterDelta = 0.01;

    public static double pidFShooter = 0.0024; // rpm -> proportion of max RPM at 1V
    public static double pidPShooter = 0.002;
    public static double pidIShooter = 0.001;
    public static double pidDShooter = 0.00015;
    public static double pidGammaShooter = 0.1;

    public static double swerveAngularVelocityCorrectionDeadzone = 0.1;

    public static double pidPHeadingEnforcement = 0.3;
    public static double pidIHeadingEnforcement = 0;
    public static double pidDHeadingEnforcement = 0;
    public static double pidTauHeadingEnforcement = 1;
    public static double pidGammaHeadingEnforcement = 3000;
    public static double headingEnforcementLateralForceCutoff = 0.3;

    public static double optimalGateIntakeAngle = 2.20;

    public static int turretShootOnTheMoveConvergenceIterations = 5;
}
