package org.firstinspires.ftc.teamcode.subsystems.localizer;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;

public interface Localizer extends org.beaverbots.beavertracking.Localizer {
    DrivetrainState getPosition();
    DrivetrainState getVelocity();

    static double wind(double theta, double to) {
        double TWO_PI = 2.0 * Math.PI;
        double delta = theta - to;
        double normalizedDelta = delta - TWO_PI * Math.floor((delta + Math.PI) / TWO_PI);
        return to + normalizedDelta;
    }

    double wind(double theta);
}
