package org.firstinspires.ftc.teamcode;

import java.util.List;

public final class Constants {
    public static final double ROBOT_LENGTH = 15;
    public static final double ROBOT_WIDTH = 15;

    public static final double GOAL_X = -70.160065 + 5;
    public static final double GOAL_Y = 70.191315 - 5;

    public static final double BASE_X = 37.71875;
    public static final double BASE_Y = 32.96875;

    public static final List<Double> SHOOTING_ZONE_NEAR_X = List.of(-70.468750, 0.0, -70.478750);
    public static final List<Double> SHOOTING_ZONE_NEAR_Y = List.of(-69.949143, 0.0, 69.499143);

    public static final List<Double> SHOOTING_ZONE_FAR_X = List.of(69.636643, 46.386643, 69.636643);
    public static final List<Double> SHOOTING_ZONE_FAR_Y = List.of(-23.218750, 0.0, 23.218750);

    public static double drivetrainPowerConversionFactorX = 1 / 62.45693318101283 / (1 - 0.1);
    public static double drivetrainPowerConversionFactorY = 1 / 62.45693318101283 / (1 - 0.3);
    public static double drivetrainPowerConversionFactorTheta = 1 / 6.874573388870918;

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
    public static double pidDX = 1.46652 / 4;
    public static double pidTauX = 0.1;
    public static double pidGammaX = 300;
    public static double pidPY = 4.6;
    public static double pidIY = 13.142857142857146;
    public static double pidDY = 1.073226 / 4;
    public static double pidTauY = 0.1;
    public static double pidGammaY = 300;
    public static double pidPTheta = 3.6;
    public static double pidITheta = 20.571428571428573;
    public static double pidDTheta = 0.419958 / 4;
    public static double pidTauTheta = 0.1;
    public static double pidGammaTheta = 3000;

    /*
    public static double pidPX = 3;
    public static double pidIX = 10;
    public static double pidDX = 1 / 2.;
    public static double pidTauX = 0.1;
    public static double pidGammaX = 1000;
    public static double pidPY = 3.4;
    public static double pidIY = 10;
    public static double pidDY = 1.2 / 2.;
    public static double pidTauY = 0.1;
    public static double pidGammaY = 1000;
    public static double pidPTheta = 4;
    public static double pidITheta = 2;
    public static double pidDTheta = 0.3 / 2.;
    public static double pidTauTheta = 0.1;
    public static double pidGammaTheta = 10000;

     */

/*
    public static double pidPX = 3;
    public static double pidIX = 10;
    public static double pidDX = 1 / 2.;
    public static double pidTauX = 0.1;
    public static double pidGammaX = 1000;
    public static double pidPY = 3.4;
    public static double pidIY = 10;
    public static double pidDY = 1.2 / 2.;
    public static double pidTauY = 0.1;
    public static double pidGammaY = 1000;
    public static double pidPTheta = 4;
    public static double pidITheta = 2;
    public static double pidDTheta = 0.3 / 2.;
    public static double pidTauTheta = 0.1;
    public static double pidGammaTheta = 10000;
 */

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
    public static double pinpointXOffset = -3.652766;
    // Forward of center is positive, behind center is negative
    public static double pinpointYOffset = -3.775394;


    public static final double turretAngularBias = -Math.toRadians(2);

    public static double pidFShooter = 0.0023; // rpm -> proportion of max RPM at 1V
    public static double pidPShooter = 0.001;
    public static double pidIShooter = 0.001;
    public static double pidGammaShooter = 5;

    public static double lateralVariancePinpoint = 0.000683634022087;
    public static double angularVariancePinpoint = 0.0000000268388027135 * 1000;

    public static double minLateralVariance = 0;
    public static double minAngularVariance = 0;

    public static double turretLatency = 0.08;
    public static double turretBounds = 5 * Math.PI / 9;
}
