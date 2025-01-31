package org.firstinspires.ftc.teamcode.pathfollower2;

import java.util.HashMap;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class PathSegment {
  public HashMap<DOFs.DOF, Function<Double, Double>> f;
  public Function<Double, Boolean> isFinished;
  public Runnable onInit;
  public Runnable onIteration;
  public Runnable onInitBlocking;
  public Runnable onIterationBlocking;

  public PathSegment(HashMap<DOFs.DOF, Function<Double, Double>> f, Function<Double, Boolean> isFinished, Runnable onInit, Runnable onIteration, Runnable onInitBlocking, Runnable onIterationBlocking) {
    this.f = f;
    this.isFinished = isFinished;
    this.onInit = onInit;
    this.onIteration = onIteration;
    this.onInitBlocking = onInitBlocking;
    this.onIterationBlocking = onIterationBlocking;
  }
}