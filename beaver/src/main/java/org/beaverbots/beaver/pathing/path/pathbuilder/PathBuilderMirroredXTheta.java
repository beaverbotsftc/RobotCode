package org.beaverbots.beaver.pathing.path.pathbuilder;

import java.util.List;

///  Holonomic only, order: X, Y, Theta
public class PathBuilderMirroredXTheta extends PathBuilder {
    public PathBuilderMirroredXTheta(List<Double> startingPosition) {
        super(startingPosition, java.util.Arrays.asList(x -> 144 - x, y -> y, theta -> -theta), true);
    }
}