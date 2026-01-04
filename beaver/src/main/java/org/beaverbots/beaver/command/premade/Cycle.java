package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandGroup;

public class Cycle extends CommandGroup {
    private boolean newCommandStart;

    private int index;

    public Cycle(Command... commands) {
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
        int startIndex = index;

        while (!commands.isEmpty()) {
            Command current = commands.get(index);

            if (newCommandStart) {
                current.start();
                newCommandStart = false;
            }

            if (current.periodic()) {
                current.stop();
                index += 1;
                index %= commands.size();
                newCommandStart = true;

                if (index == startIndex) break;
            }
        }

        return false;
    }

    @Override
    public void stop() {
        if (!commands.isEmpty())
            commands.get(0).stop();
    }
}
