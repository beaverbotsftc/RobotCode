package org.beaverbots.BeaverCommand;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public abstract class CommandGroup implements Command {
    protected final List<Command> commands;

    public CommandGroup(Command... commands) {
        if (commands.length == 0) throw new IllegalArgumentException("Must have at least one command");
        this.commands = new ArrayList<>(Arrays.asList(commands));
    }

    public final Set<Subsystem> getDependencies() {
        Set<Subsystem> dependencies = new HashSet<>();
        for (Command command : commands) {
            // Command.calculateDependencies will still work, but this provides a more minimal set.
            dependencies.addAll(command.getDependencies());
        }
        return dependencies;
    }
}
