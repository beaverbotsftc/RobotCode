package org.beaverbots.BeaverCommand.util;

import org.beaverbots.BeaverCommand.Command;

import java.util.function.BooleanSupplier;

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
