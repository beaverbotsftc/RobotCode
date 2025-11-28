package org.beaverbots.BeaverCommand.util;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandGroup;

public class Sequential extends CommandGroup {
    private boolean newCommandStart = true;

    public Sequential(Command... commands) {
        super(commands);
    }

    @Override
    public boolean periodic() {
        if (newCommandStart)
            commands.get(0).start();

        if (commands.get(0).periodic()) {
            commands.get(0).stop();
            commands.remove(0);
            newCommandStart = true;
        }

        return commands.isEmpty();
    }

    @Override
    public void stop() {
        if (!commands.isEmpty())
            commands.get(0).stop();
    }
}
