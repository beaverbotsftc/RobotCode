package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;
import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public abstract class CommandGroup implements Command {
    protected List<Command> commands;

    private final List<Command> initialCommands;

    public CommandGroup(Command... commands) {
        if (commands.length == 0)
            throw new IllegalArgumentException("No commands were provided to the CommandGroup.");

        this.initialCommands = new ArrayList<>(Arrays.asList(commands));
        this.commands = new ArrayList<>(initialCommands);
    }

    public final Set<Subsystem> getImmediateDependencies() {
        Set<Subsystem> dependencies = new HashSet<>();
        for (Command command : initialCommands) {
            // Command.calculateDependencies will still work, but this provides a more minimal set.
            dependencies.addAll(command.getImmediateDependencies());
        }
        return dependencies;
    }

    @Override
    public void start() {
        commands = new ArrayList<>(initialCommands);
    }
}
