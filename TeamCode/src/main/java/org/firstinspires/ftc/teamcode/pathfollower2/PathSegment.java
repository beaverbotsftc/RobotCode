package org.firstinspires.ftc.teamcode.pathfollower2;

import java.util.HashMap;
import java.util.function.Function;

public class PathSegment {
  public HashMap<DOFs.DOF, Function<Double, Double>> f;
  public Function<Double, Boolean> isFinished;

  public PathSegment(HashMap<DOFs.DOF, Function<Double, Double>> f, Function<Double, Boolean> isFinished) {
    this.f = f;
    this.isFinished = isFinished;
  }

}