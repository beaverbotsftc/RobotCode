package org.beaverbots.BeaverCommand.util;

import org.beaverbots.BeaverCommand.Command;

import java.util.function.BooleanSupplier;

public class WaitUntil implements Command {
    BooleanSupplier f;

    public WaitUntil(BooleanSupplier f) {
        this.f = f;
    }

    @Override
    public boolean periodic() {
        return f.getAsBoolean();
    }
}
