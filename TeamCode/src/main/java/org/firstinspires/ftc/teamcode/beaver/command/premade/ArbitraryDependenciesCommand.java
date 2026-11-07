package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;
import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public abstract class ArbitraryDependenciesCommand implements Command {
    private final Set<Subsystem> dependencies;

    public ArbitraryDependenciesCommand(Subsystem... dependencies) {
        this.dependencies = new HashSet<>(Arrays.asList(dependencies));
    }

    @Override
    public final Set<Subsystem> getImmediateDependencies() {
        return dependencies;
    }
}
