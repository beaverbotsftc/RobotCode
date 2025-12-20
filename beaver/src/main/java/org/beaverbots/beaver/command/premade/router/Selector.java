package org.beaverbots.beaver.command.premade.router;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class Selector {
    private final IntSupplier f;

    public int getIndex() {
        return f.getAsInt();
    }

    public Selector(IntSupplier f) {
        this.f = f;
    }

    /// 1 if true, 0 if false
    public Selector(BooleanSupplier f) {
        this.f = () -> f.getAsBoolean() ? 1 : 0;
    }
}
