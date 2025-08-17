package org.firstinspires.ftc.teamcode.beaverbotv0.pathfollower;

public final class Feedforward {
    public static class K {
        public double kineticFriction;
        public double acceleration;
    }

    private K k;

    public Feedforward(K k) {
        this.k = k;
    }

    public double prediction(double velocity, double acceleration) {
        return k.kineticFriction * velocity + k.acceleration * acceleration;
    }
}