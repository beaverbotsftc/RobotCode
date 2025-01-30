package org.firstinspires.ftc.teamcode.pathfollower2;

import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class PathSegment {
  public HashMap<DOFs.DOF, Function<Double, Double>> f;
  public Function<Double, Boolean> isFinished;
  public Consumer<Supplier<Boolean>> init;

  public PathSegment(HashMap<DOFs.DOF, Function<Double, Double>> f, Function<Double, Boolean> isFinished, Consumer<Supplier<Boolean>> init) {
    this.f = f;
    this.isFinished = isFinished;
    this.init = init;
  }
}