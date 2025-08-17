package org.firstinspires.ftc.teamcode.pathfollowerv3;

import java.util.HashMap;
import java.util.function.Function;

public class PathSegment {
  public HashMap<Robot.DOF, Function<Double, Double>> f;
  public Function<Double, Boolean> isFinished;
  public Runnable onInit;
  public Runnable onTick;
  public Runnable onInitBlocking;

  public PathSegment(HashMap<Robot.DOF, Function<Double, Double>> f, Function<Double, Boolean> isFinished, Runnable onInit, Runnable onTick, Runnable onInitBlocking) {
    this.f = f;
    this.isFinished = isFinished;
    this.onInit = onInit;
    this.onTick = onTick;
    this.onInitBlocking = onInitBlocking;
  }
}