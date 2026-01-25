package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandGroup;

import java.util.function.Supplier;

public class Defer implements Command {
    private Supplier<Command> commandSupplier;
    private Command command;

    public Defer(Supplier<Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
    }

    @Override
    public void start() {
        command = commandSupplier.get();
        command.start();
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
