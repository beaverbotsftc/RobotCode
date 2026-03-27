package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;

public class StopWith implements Command {
    private Command command;
    private Runnable stop;

    public StopWith(Command command, Runnable stop) {
        this.command = command;
        this.stop = stop;
    }

    @Override
    public void start() {
        command.start();
    }

    @Override
    public boolean periodic() {
        return command.periodic();
    }

    @Override
    public void stop() {
        command.stop();
        stop.run();
    }
}
