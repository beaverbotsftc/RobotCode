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
            // TODO: If so, continue here.
            if (dependencies.contains(dependency)) throw new CircularDependencyException(String.format("%s requires %s, which (potentially indirectly) requires %s.", self, dependency, self));
            Subsystem.calculateDependencies(dependency, dependencies);
        }
    }
}