package org.firstinspires.ftc.teamcode.beaver.command;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public interface Subsystem {
    /// This should not include read-only access;
    /// only other subsystems this physically influences should be reported.
    default Set<Subsystem> getDependencies() { return Collections.emptySet(); }

    default void periodic() {}

    static Set<Subsystem> calculateDependencies(Subsystem self) {
        Set<Subsystem> dependencies = new HashSet<>();
        calculateDependencies(self, dependencies);
        return dependencies;
    }

    static void calculateDependencies(Subsystem self, Set<Subsystem> dependencies) {
        dependencies.add(self);
        for (Subsystem dependency : self.getDependencies()) {
            // No possible subsystem arrangement can form something like this;
            // some acyclic structures are caught, but those structures are dangerous even without a cycle.
            if (dependencies.contains(dependency))
                throw new DependencyException(String.format("There is a potential dependency error involving %s and %s.", self, dependency));
            Subsystem.calculateDependencies(dependency, dependencies);
        }
    }
}
