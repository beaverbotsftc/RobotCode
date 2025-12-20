package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandGroup;

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
        while (!commands.isEmpty()) {
            Command current = commands.get(0);

            if (newCommandStart) {
                current.start();
                newCommandStart = false;
            }

            if (current.periodic()) {
                current.stop();
                commands.remove(0);
                newCommandStart = true;
            } else {
                return false;
            }
        }
        return true; // All commands finished
    }

    @Override
    public void stop() {
        if (!commands.isEmpty())
            commands.get(0).stop();
    }
}
