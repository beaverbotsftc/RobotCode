package org.firstinspires.ftc.teamcode.beaver.util;

import java.util.Arrays;

public class Beaver {
    public static <T> T[] concat(T[] first, T[] second) {
        T[] result = Arrays.copyOf(first, first.length + second.length);
        System.arraycopy(
                second, 0,
                result, first.length,
                second.length
        );
        return result;
    }

    public static <T> T[] prepend(T element, T[] array) {
        T[] result = Arrays.copyOf(array, array.length + 1);
        System.arraycopy(array, 0, result, 1, array.length);
        result[0] = element;
        return result;
    }

    public static <T> T[] append(T element, T[] array) {
        T[] result = Arrays.copyOf(array, array.length + 1);
        result[array.length] = element;
        return result;
    }

    public static double modulus(double n, double mod) {
        return n - Math.floor(n / mod) * mod;
    }
}
