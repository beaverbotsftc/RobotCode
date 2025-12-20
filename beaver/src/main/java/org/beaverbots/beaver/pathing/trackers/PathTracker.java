package org.beaverbots.beaver.pathing.trackers;

import java.util.List;

public interface PathTracker {
    List<Double> update(List<Double> position, double dt);
}
