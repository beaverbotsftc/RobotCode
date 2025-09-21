package org.beaverbots.beavertracking;

import java.util.List;

public interface PathTracker {
    List<Double> update(List<Double> position, double dt);
}
