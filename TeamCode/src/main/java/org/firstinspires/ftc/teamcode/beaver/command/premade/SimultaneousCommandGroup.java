package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;
import org.firstinspires.ftc.teamcode.beaver.command.ConflictingCommandsException;
import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public abstract class SimultaneousCommandGroup extends CommandGroup {
    public SimultaneousCommandGroup(Command... commands) {
        super(commands);
        checkDependencies();
    }

    protected void checkDependencies() {
        Set<Subsystem> dependencies = new HashSet<>();
        for (Command command : commands) {
            Set<Subsystem> commandDependencies = Command.calculateDependencies(command);
            if (!Collections.disjoint(dependencies, commandDependencies)) {
                throw new ConflictingCommandsException(String.format("Command '%s' has dependencies not disjoint with the rest of the command group.", command));
            }
            dependencies.addAll(commandDependencies);
        }
    }
}
