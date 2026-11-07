package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;
import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;

import java.util.Collections;
import java.util.Set;

public class NoOp implements Command {
    public NoOp() {}

    @Override
    public Set<Subsystem> getImmediateDependencies() { return Collections.emptySet(); }

    @Override
    public boolean periodic() {
        return true;
    }
}