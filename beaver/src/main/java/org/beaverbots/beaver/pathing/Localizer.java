package org.beaverbots.beaver.pathing;

import org.beaverbots.beaver.command.Subsystem;

import java.util.List;

public interface Localizer extends Subsystem {
    List<Double> getPositionAsList();

    List<Double> getVelocityAsList();
}
