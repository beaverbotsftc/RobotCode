package org.firstinspires.ftc.teamcode.subsystems;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.BeaverCommand.Subsystem;
import org.beaverbots.BeaverSensor.UnscentedKalmanFilter;

import java.util.Set;

public class Localization implements Subsystem {
    private Pinpoint pinpoint;
    private Limelight limelight;

    private UnscentedKalmanFilter filter;

    public Set<Subsystem> getDependencies() {
        return Set.of(limelight);
    }

    public Localization(Pinpoint pinpoint, Limelight limelight) {
        this.pinpoint = pinpoint;
        this.limelight = limelight;
        // alpha is sigma point spread
        this.filter = new UnscentedKalmanFilter(
                3,
                new ArrayRealVector(pinpoint.getPosition().toArray()),
                new Array2DRowRealMatrix(new double[][]{{3.87499225}}),
                0.1,
                (RealVector state, RealVector control, double dt) -> new ArrayRealVector(pinpoint.getPosition().toArray()),
                new Array2DRowRealMatrix(new double[][]{{0.1}}));
    }
}
