package org.beaverbots.beaver.util;

public final class Modulus {
    public static double modulus(double a, double b) {
        double remainder = a % b;
        if (remainder < 0) {
            return b + remainder;
        }
        return remainder;
    }
}
