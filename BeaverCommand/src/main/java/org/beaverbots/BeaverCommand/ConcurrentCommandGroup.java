// ./ConcurrentCommandGroup.java

package org.beaverbots.BeaverCommand;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

public class ConcurrentCommandGroup extends CommandGroup {
    private final List<Command> runningCommands = new ArrayList<>();

    public ConcurrentCommandGroup(Command... commands) {
        super(commands);
        checkDependencies();
    }

    private void checkDependencies() {
        Set<Subsystem> allDependencies = new HashSet<>();
        for (Command command : commands) {
            Set<Subsystem> commandDependencies = Command.calculateDependencies(command);
            if (!Collections.disjoint(allDependencies, commandDependencies)) {
                throw new ConflictingCommandsException(
                        String.format("Command '%s' in ConcurrentCommandGroup has dependencies that conflict with other commands in the same group.", command)
                );
            }
            allDependencies.addAll(commandDependencies);
        }
    }

    @Override
    public void start() {
        runningCommands.clear();
        runningCommands.addAll(commands);

        for (Command command : runningCommands) {
            command.start();
        }
    }

    @Override
    public boolean periodic() {
        Iterator<Command> iterator = runningCommands.iterator();
        while (iterator.hasNext()) {
            Command command = iterator.next();
            if (command.periodic()) {
                command.stop();
                iterator.remove();
            }
        }

        return runningCommands.isEmpty();
    }

    @Override
    public void stop() {
        for (Command command : runningCommands) {
            command.stop();
        }
    }
}