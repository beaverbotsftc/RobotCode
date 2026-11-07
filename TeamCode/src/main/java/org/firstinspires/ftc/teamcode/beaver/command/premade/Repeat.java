package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;
import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;

import java.util.Collections;
import java.util.Set;

public class Repeat extends ArbitraryDependenciesCommand {
    private final Runnable f;

    public Repeat(Runnable f, Subsystem... subsystems) {
        super(subsystems);
        this.f = f;
    }

    @Override
    public boolean periodic() {
        f.run();
        return false;
    }
}
