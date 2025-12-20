package org.beaverbots.beaver.command.premade.router;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandGroup;

public class Router extends CommandGroup {
    private final Selector selector;

    private int lastSelectedIndex = -1;

    private int runningIndex = -1;

    public Router(Selector selector, Command... commands) {
        super(commands);
        if (commands.length == 0) {
            throw new IllegalArgumentException("Must provide at least one command");
        }
        this.selector = selector;
    }

    @Override
    protected void checkDependencies() {
        // Router intentionally allows shared dependencies (mutually exclusive)
    }

    @Override
    public boolean periodic() {
        int desiredIndex = selector.getIndex();
        if (desiredIndex < 0 || desiredIndex >= commands.size()) {
            throw new IllegalArgumentException("Index out of range: " + desiredIndex);
        }

        if (desiredIndex != lastSelectedIndex) {
            if (runningIndex != -1) {
                commands.get(runningIndex).stop();
            }

            commands.get(desiredIndex).start();
            runningIndex = desiredIndex;
        } else {
            if (runningIndex != -1) {
                if (commands.get(runningIndex).periodic()) {
                    commands.get(runningIndex).stop();
                    runningIndex = -1;
                }
            }
        }

        lastSelectedIndex = desiredIndex;

        return false;
    }

    @Override
    public void stop() {
        if (runningIndex != -1) {
            commands.get(runningIndex).stop();
        }
        runningIndex = -1;
        lastSelectedIndex = -1;
    }
}
