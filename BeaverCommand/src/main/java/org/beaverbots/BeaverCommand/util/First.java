package org.beaverbots.BeaverCommand.util;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandGroup;

public class First extends CommandGroup {
    public static final boolean isLoud = true;

    public First(Command... commands) {
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
        for (int i = commands.size() - 1; i >= 0; i--) {
            Command command = commands.get(i);
            if (command.periodic())
                return true;
        }

        return false;
    }

    @Override
    public void stop() {
        for (Command command : commands) {
            command.stop();
        }
    }
}