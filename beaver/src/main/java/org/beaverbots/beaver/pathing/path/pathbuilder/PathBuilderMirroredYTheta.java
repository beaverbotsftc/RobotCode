package org.beaverbots.beaver.pathing.path.pathbuilder;

import java.util.List;

///  Holonomic only, order: X, Y, Theta
public class PathBuilderMirroredYTheta extends PathBuilder {
    public PathBuilderMirroredYTheta(List<Double> startingPosition) {
        super(startingPosition, java.util.Arrays.asList(x -> x, y -> 144 - y, theta -> -theta), true);
    }

    public PathBuilderMirroredYTheta(List<Double> startingPosition, boolean doubleTransformStart) {
        super(startingPosition, java.util.Arrays.asList(x -> x, y -> 144 - y, theta -> -theta), doubleTransformStart);
    }
}