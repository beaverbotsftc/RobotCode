package org.beaverbots.beaver.pathing;

import org.beaverbots.beaver.command.Subsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public interface Locomotion extends Subsystem {
    void move(List<Double> velocity, List<Double> position);
    default void move(List<Double> velocity) {
        move(velocity, new ArrayList<>(Collections.nCopies(10, 0.0)));
    } // Local
}
