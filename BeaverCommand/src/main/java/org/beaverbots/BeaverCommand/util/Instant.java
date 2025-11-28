package org.beaverbots.BeaverCommand.util;

import org.beaverbots.BeaverCommand.Command;

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
