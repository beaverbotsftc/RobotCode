package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandGroup;

public class Parallel extends CommandGroup {
    public Parallel(Command... commands) {
        super(commands);
    }

    @Override
    public void start() {
        super.start();

        for (Command command : commands) {
            command.start();
        }
    }

    @Override
    public boolean periodic() {
        for (int i = 0; i < commands.size(); i++) {
            Command command = commands.get(i);
            if (command.periodic()) {
                command.stop();
                commands.remove(i);
                i--;
            }
        }
        return commands.isEmpty();
    }

    @Override
    public void stop() {
        for (Command command : commands) {
            command.stop();
        }
    }
}