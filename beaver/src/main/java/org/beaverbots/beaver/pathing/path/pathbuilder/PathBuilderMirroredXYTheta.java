package org.beaverbots.beaver.pathing.path.pathbuilder;

import java.util.List;

///  Holonomic only, order: X, Y, Theta
public class PathBuilderMirroredXYTheta extends PathBuilder {
    public PathBuilderMirroredXYTheta(List<Double> startingPosition) {
        super(startingPosition, java.util.Arrays.asList(x -> 144 - x, y -> 144 - y, theta -> -theta), true);
    }
}