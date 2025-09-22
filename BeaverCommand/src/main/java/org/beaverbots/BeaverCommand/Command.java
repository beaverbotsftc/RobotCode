package org.beaverbots.BeaverCommand;

import java.util.HashSet;
import java.util.Set;

public interface Command {
    default Set<Subsystem> getDependencies() {
        return new HashSet<>();
    }

    ///  Returns true if and only if the Command finished. Otherwise, it shall return false.
    ///  To call periodic before start is undefined behavior
    boolean periodic();

    ///  Is called before either periodic or stop is called
    ///  However, command reuse is okay and therefore it must handle that
    default void start() {}

    ///  To call stop before start is undefined behavior
    default void stop() {}

    static Set<Subsystem> calculateDependencies(Command self) {
        Set<Subsystem> fullDependencies = new HashSet<>();
        for (Subsystem dependency : self.getDependencies()) {
            fullDependencies.addAll(Subsystem.calculateDependencies(dependency));
        }
        return fullDependencies;
    }
}
