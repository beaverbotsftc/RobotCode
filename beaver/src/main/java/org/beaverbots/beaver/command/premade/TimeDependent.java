package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;

public class TimeDependent implements Command {
    Runnable f;

    public TimeDependent(Runnable f) {
        this.f = f;
    }

    @Override
    public boolean periodic() {
        f.run();
        return false;
    }
}
