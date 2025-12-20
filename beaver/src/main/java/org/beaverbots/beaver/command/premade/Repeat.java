package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;

public class Repeat implements Command {
    Runnable f;

    public Repeat(Runnable f) {
        this.f = f;
    }

    @Override
    public boolean periodic() {
        f.run();
        return false;
    }
}
