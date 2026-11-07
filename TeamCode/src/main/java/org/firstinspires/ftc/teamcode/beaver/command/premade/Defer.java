package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;
import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;

import java.util.Collections;
import java.util.Set;
import java.util.function.Supplier;

public class Defer implements Command {
    private Supplier<Command> commandSupplier;
    private Command command;
    private Set<Subsystem> dependencies;

    ///  Note: Instantly calls commandSupplier for dependency resolution, and then also calls it again later. Explicitly pass dependencies if an NPE would occur.
    public Defer(Supplier<Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
        dependencies = commandSupplier.get().getImmediateDependencies();
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
    public Set<Subsystem> getImmediateDependencies() {
        return dependencies;
    }

    @Override
    public boolean periodic() {
        return command.periodic();
    }

    @Override
    public void stop() {
        if (command != null) command.stop();
    }
}
