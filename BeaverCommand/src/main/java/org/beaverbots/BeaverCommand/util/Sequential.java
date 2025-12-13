package org.beaverbots.BeaverCommand.util;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandGroup;

public class Sequential extends CommandGroup {
    private boolean newCommandStart;

    public Sequential(Command... commands) {
        super(commands);
    }

    @Override
    public void start() {
        super.start();

        newCommandStart = true;
    }

    @Override
    protected void checkDependencies() {
        // Sequential commands run one at a time, so they are allowed to share dependencies.
    }

    @Override
    public boolean periodic() {
        if (commands.isEmpty()) return true;

        if (newCommandStart) commands.get(0).start();
        newCommandStart = false;

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
