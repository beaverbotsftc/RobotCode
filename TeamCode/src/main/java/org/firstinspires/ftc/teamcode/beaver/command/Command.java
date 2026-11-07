package org.firstinspires.ftc.teamcode.beaver.command;

import java.util.HashSet;
import java.util.Set;

public interface Command {
    Set<Subsystem> getImmediateDependencies();

    ///  Returns true if the Command finished. Otherwise, returns false.
    boolean periodic();

    default void start() {}
    default void stop() {}

    static Set<Subsystem> calculateDependencies(Command self) {
        Set<Subsystem> fullDependencies = new HashSet<>();
        for (Subsystem dependency : self.getImmediateDependencies()) {
            fullDependencies.addAll(Subsystem.calculateDependencies(dependency));
        }
        return fullDependencies;
    }
}
