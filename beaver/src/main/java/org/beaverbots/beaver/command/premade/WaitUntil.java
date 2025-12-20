package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;

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
