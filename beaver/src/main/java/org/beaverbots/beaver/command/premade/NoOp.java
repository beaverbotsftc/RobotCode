package org.beaverbots.beaver.command.premade;

import org.beaverbots.beaver.command.Command;

public class NoOp implements Command {
    public NoOp() {}

    @Override
    public boolean periodic() {
        return true;
    }
}
