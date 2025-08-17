package org.firstinspires.ftc.teamcode.pathfollowerv3;

import java.util.HashMap;
import java.util.function.Function;

public class PathSegment {
  public HashMap<Robot.DOF, Function<Double, Double>> f;
  public Function<Double, Boolean> isFinished;

  /// Assumes that onInit is thread safe, as it runs in it's own thread.
  public PathSegment(HashMap<Robot.DOF, Function<Double, Double>> f, Function<Double, Boolean> isFinished) {
    this.f = f;
    this.isFinished = isFinished;
  }
}