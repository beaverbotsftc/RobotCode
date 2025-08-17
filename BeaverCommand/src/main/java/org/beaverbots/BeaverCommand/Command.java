package org.beaverbots.BeaverCommand;

import java.util.HashSet;
import java.util.Set;

public interface Command {
    default Set<Subsystem> getDependencies() {
        return new HashSet<>();
    }

    ///  Returns true if and only if the Command finished. Otherwise, it shall return false.
    boolean periodic();

    default void start() {}
    default void stop() {}

    static Set<Subsystem> calculateDependencies(Command self) {
        Set<Subsystem> fullDependencies = new HashSet<>();
        for (Subsystem dependency : self.getDependencies()) {
            fullDependencies.addAll(Subsystem.calculateDependencies(dependency));
        }
        return fullDependencies;
    }
}
