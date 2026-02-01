package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;

public class NoOp implements Command {
    Runnable f;

    public NoOp(Runnable f) {
        this.f = f;
    }

    @Override
    public boolean periodic() {
        f.run();
        return true;
    }
}
