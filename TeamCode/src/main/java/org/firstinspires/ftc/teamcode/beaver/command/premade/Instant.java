package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;

public class Instant extends ArbitraryDependenciesCommand {
    private final Runnable f;

    public Instant(Runnable f, Subsystem... subsystems) {
        super(subsystems);
        this.f = f;
    }

    @Override
    public boolean periodic() {
        f.run();
        return true;
    }
}