package org.beaverbots.beavertracking;

import org.beaverbots.BeaverCommand.Subsystem;

import java.util.List;

public interface Locomotion extends Subsystem {
    void move(List<Double> movement);
}
