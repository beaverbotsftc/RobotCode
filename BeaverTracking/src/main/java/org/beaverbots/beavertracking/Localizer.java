package org.beaverbots.beavertracking;

import org.beaverbots.BeaverCommand.Subsystem;

import java.util.List;

public interface Localizer extends Subsystem {
    List<Double> getPosition();

    List<Double> getVelocity();
}
