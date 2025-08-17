package org.beaverbots.BeaverCommand;

import java.util.HashSet;
import java.util.Set;

public interface Subsystem {
    default Set<Subsystem> getDependencies() {
        return new HashSet<>();
    }

    default void periodic() {}

    ///  This runs after periodic, if no command is using it.
    default void periodicDefault() {}

    static Set<Subsystem> calculateDependencies(Subsystem self) {
        Set<Subsystem> dependencies = new HashSet<>();
        calculateDependencies(self, dependencies);
        return dependencies;
    }

    static void calculateDependencies(Subsystem self, Set<Subsystem> dependencies) {
        dependencies.add(self);
        for (Subsystem dependency : self.getDependencies()) {
            // TODO: Evaluate whether recursive dependencies is illegal or not / has any valid use case.
            // TODO: If not, throw an exception here.
            if (dependencies.contains(dependency)) continue;
            Subsystem.calculateDependencies(dependency, dependencies);
        }
    }
}