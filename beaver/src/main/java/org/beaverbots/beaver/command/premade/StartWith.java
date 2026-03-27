package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;

public class StartWith implements Command {
    private Command command;
    private Runnable start;

    public StartWith(Command command, Runnable start) {
        this.command = command;
        this.start = start;
    }

    @Override
    public void start() {
        start.run();
        command.start();
    }

    @Override
    public boolean periodic() {
        return command.periodic();
    }

    @Override
    public void stop() {
        command.stop();
    }
}
