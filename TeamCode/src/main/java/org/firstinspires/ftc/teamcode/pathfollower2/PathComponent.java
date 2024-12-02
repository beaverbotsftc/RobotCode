package org.firstinspires.ftc.teamcode.pathfollower2;


import java.util.HashMap;
import java.util.function.Function;

public class PathComponent {
    public Function<Double, Boolean> isFinished;
    public HashMap<DOFs.DOF, Function<Double, Double>> getPositions;

    public PathComponent(Function<Double, Boolean> isFinished, HashMap<DOFs.DOF, Function<Double, Double>> getPositions) {
        this.isFinished = isFinished;
        this.getPositions = getPositions;
    }
}
