package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandGroup;

import java.util.function.IntPredicate;

public class Cycle extends CommandGroup {
    private IntPredicate isFinished;
    private boolean newCommandStart;
    private int iterations = 0;

    private int index;

    public Cycle(IntPredicate isFinished, Command... commands) {
        super(commands);
        this.isFinished = isFinished;
    }

    @Override
    public void start() {
        iterations = 0;
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
                if (index == commands.size()) iterations++;
                index %= commands.size();
                newCommandStart = true;

                if (index == startIndex) break;
            } else break;
        }

        return isFinished.test(iterations);
    }

    @Override
    public void stop() {
        if (!commands.isEmpty())
            commands.get(index).stop();
    }
}
