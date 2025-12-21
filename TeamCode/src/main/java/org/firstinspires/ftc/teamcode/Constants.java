package org.firstinspires.ftc.teamcode;

public final class Constants {
    public static double drivetrainPowerConversionFactorX = 0.0128674126; // 0.01072290598019666; // in/s -> proportion of max rpm
    public static double drivetrainPowerConversionFactorY = 0.0142546161; // 0.01072290598019666; // in/s -> proportion of max rpm
    public static double drivetrainPowerConversionFactorTheta = 0.1603526706; // 0.1319953842421724; // rad/s -> proportion of max rpm

    public static double getMaxLateralVelocity() {
        return Math.min(1 / drivetrainPowerConversionFactorX, 1 / drivetrainPowerConversionFactorY);
    }

    public static double getMaxAngularVelocity() {
        return 1 / drivetrainPowerConversionFactorTheta;
    }

    /*
    // Ziegler–Nichols method
    // K_p = 0.2 * K_u
    // K_i = 0.4 * K_u / T_u
    // K_d = 0.06666 * K_u * T_u

    // Ku_X = 22
    // Tu_X = 1
    // Ku_Y = 23
    // Tu_Y = 0.7
    // Ku_Theta = 18
    // Tu_Theta = 0.35

    public static double pidPX = 4.4;
    public static double pidIX = 8.8;
    public static double pidDX = 1.46652;
    public static double pidTauX = 0.1;
    public static double pidPY = 4.6;
    public static double pidIY = 13.142857142857146;
    public static double pidDY = 1.073226;
    public static double pidTauY = 0.1;
    public static double pidPTheta = 3.6;
    public static double pidITheta = 20.571428571428573;
    public static double pidDTheta = 0.41995799999999994;
    public static double pidTauTheta = 0.1;
*/


    public static double pidPX = 3;
    public static double pidIX = 0;
    public static double pidDX = 1;
    public static double pidTauX = 0.1;
    public static double pidGammaX = 0;
    public static double pidPY = 3.4;
    public static double pidIY = 0;
    public static double pidDY = 1.2;
    public static double pidTauY = 0.1;
    public static double pidGammaY = 0;
    public static double pidPTheta = 4;
    public static double pidITheta = 0;
    public static double pidDTheta = 0.3;
    public static double pidTauTheta = 0.1;
    public static double pidGammaTheta = 0;

    /*
    public static double pidPX = 0;
    public static double pidIX = 0;
    public static double pidDX = 0;
    public static double pidPY = 0;
    public static double pidIY = 0;
    public static double pidDY = 0;
    public static double pidPTheta = 0;
    public static double pidITheta = 0;
    public static double pidDTheta = 0;
     */

    // Left of center is positive, right of center is negative
    public static double pinpointXOffset = -4;
    // Forward of center is positive, behind center is negative
    public static double pinpointYOffset = -2.5;

    public static final double redGoalX = 144 - 5;
    public static final double redGoalY = 0 + 5;
    public static final double blueGoalX = 144 - 5;
    public static final double blueGoalY = 144 - 5;

    public static final double shooterBias = 0.00;

    public static double shooterFrictionConversionFactor = 0.00258; // rpm -> proportion of max RPM at 1V
    public static double pidPShooter = 0.003;
    public static double pidIShooter = 0.003;
    public static double pidGammaShooter = 0.0004;
}
