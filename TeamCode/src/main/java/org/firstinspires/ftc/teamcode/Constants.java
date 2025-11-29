package org.firstinspires.ftc.teamcode;

public final class Constants {
    public static double drivetrainPowerConversionFactorX = 0.0128674126; // 0.01072290598019666; // in/s -> proportion of max rpm
    public static double drivetrainPowerConversionFactorY = 0.0142546161; // 0.01072290598019666; // in/s -> proportion of max rpm
    public static double drivetrainPowerConversionFactorTheta = 0.1603526706; // 0.1319953842421724; // rad/s -> proportion of max rpm

    /*
    // Ziegler–Nichols method
    // K_p = 0.2 * K_u
    // K_i = 0.4 * K_u / T_u
    // K_d = 0.06666 * K_u * T_u

    // Ku_X = 35
    // Tu_X = 1
    // Ku_Y = 38
    // Tu_Y = 0.7
    // Ku_Theta = 30
    // Tu_Theta = 0.35

    public static double pidPX = 7;
    public static double pidIX = 14;
    public static double pidDX = 2.3333333333333333;
    public static double pidPY = 7.6;
    public static double pidIY = 21.71428571428571;
    public static double pidDY = 3.619047619047619;
    public static double pidPTheta = 6;
    public static double pidITheta = 34.28571428571429;
    public static double pidDTheta = 0.7;
    */

    public static double pidPX = 3;
    public static double pidIX = 0;
    public static double pidDX = 1;
    public static double pidTauX = 0.1;
    public static double pidPY = 3.4;
    public static double pidIY = 0;
    public static double pidDY = 1.2;
    public static double pidTauY = 0.1;
    public static double pidPTheta = 4;
    public static double pidITheta = 0;
    public static double pidDTheta = 0.3;
    public static double pidTauTheta = 0.1;

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
}
