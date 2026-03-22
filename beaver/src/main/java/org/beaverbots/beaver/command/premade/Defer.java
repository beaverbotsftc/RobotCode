package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandGroup;
import org.beaverbots.beaver.command.Subsystem;

import java.util.Set;
import java.util.function.Supplier;

public class Defer implements Command {
    private Supplier<Command> commandSupplier;
    private Command command;
    private Set<Subsystem> dependencies;

    ///  Note: Instantly calls commandSupplier for dependency resolution, and then also calls it again later. Explicitly pass dependencies if a NPE would occur.
    public Defer(Supplier<Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
        dependencies = commandSupplier.get().getDependencies();
    }

    ///  Note: Does not instantly call commandSupplier.
    public Defer(Set<Subsystem> dependencies, Supplier<Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
        this.dependencies = dependencies;
    }

    @Override
    public void start() {
        command = commandSupplier.get();
        command.start();
    }

    @Override
    public Set<Subsystem> getDependencies() {
        return dependencies;
    }

    @Override
    public boolean periodic() {
        return command.periodic();
    }

    @Override
    public void stop() {
        command.stop();
    }
}
