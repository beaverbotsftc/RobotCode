package org.firstinspires.ftc.teamcode;

public final class Constants {
    public static double pidPX = 0.04;
    public static double pidIX = 0;
    public static double pidDX = 0;
    public static double pidFVelocityX = 0.0119398561507;//0.0146;
    public static double pidFAccelerationX = 0.00510980159622;
    public static double pidGammaX = 8;
    public static double pidPY = pidPX;
    public static double pidIY = pidIX;
    public static double pidDY = pidDX;
    public static double pidFVelocityY = pidFVelocityX;
    public static double pidFAccelerationY = pidFAccelerationX;
    public static double pidGammaY = pidGammaX;
    public static double pidPTheta = 0.3;
    public static double pidITheta = 0;
    public static double pidDTheta = 0;
    public static double pidFVelocityTheta = 0.0815392296185;
    public static double pidFAccelerationTheta = 0.0292198592835;
    public static double pidGammaTheta = 64;

    public static double maxSpeedX = 1 / pidFVelocityX;
    public static double maxSpeedY = 1 / pidFVelocityY;
    public static double maxSpeedTheta = 1 / pidFVelocityTheta;
    public static double maxAccelerationX = 1 / pidFAccelerationX;
    public static double maxAccelerationY = 1 / pidFAccelerationY;
    public static double maxAccelerationTheta = 1 / pidFAccelerationTheta;

    public static double drivetrainFrontLeftFactor = 1;
    public static double drivetrainFrontRightFactor = 1;
    public static double drivetrainBackLeftFactor = 1;
    public static double drivetrainBackRightFactor = 1;



    public static double pidTau = 0.1;

    // Left of center is positive, right of center is negative
    public static double pinpointXOffset = 108.675; // mm
    // Forward of center is positive, behind center is negative
    public static double pinpointYOffset = 0.4; // mm

    public static double lateralVariancePinpoint = 0.0045660107401; // p=\left[\left(4.23,-1.07\right),\left(3.27,0.63\right),\left(-3.91,\ 0.81\right)\right after 1 minute
    public static double angularVariancePinpoint = 0.0000000268388027135 * 1000;

    public static double minLateralVariance = 0;
    public static double minAngularVariance = 0;

    public static double turretLatency = 0.08;
    public static double turretBounds = Math.toRadians(120);
    public static double turretAngularBias = 0;

    public static double turretDelta = 0.001;
    public static double shooterDelta = 0.05;

    public static double pidFShooter = 0.0025; // rpm -> proportion of max RPM at 1V
    public static double pidPShooter = 0.002;
    public static double pidIShooter = 0.001;
    public static double pidDShooter = 0.00015;
    public static double pidGammaShooter = 0.1;

    public static double headingEnforcementAngularVelocityCutoff = 0.1;

    public static double pidPGateHeading = 0.8;
    public static double pidIGateHeading = 0;
    public static double pidDGateHeading = 0;
    public static double pidTauGateHeading = 1;
    public static double pidGammaGateHeading = 3000;

    public static double pidPHeadingEnforcement = 0.1;
    public static double pidIHeadingEnforcement = 0;
    public static double pidDHeadingEnforcement = 0;
    public static double pidTauHeadingEnforcement = 1;
    public static double pidGammaHeadingEnforcement = 3000;
    public static double headingEnforcementLateralForceCutoff = 0.3;

    public static double optimalGateIntakeAngle = 2.20;

    public static int turretShootOnTheMoveConvergenceIterations = 2;
}
