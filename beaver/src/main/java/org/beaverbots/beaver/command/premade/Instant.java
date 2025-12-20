package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;

public class Instant implements Command {
    Runnable f;

    public Instant(Runnable f) {
        this.f = f;
    }

    @Override
    public boolean periodic() {
        f.run();
        return true;
    }
}
