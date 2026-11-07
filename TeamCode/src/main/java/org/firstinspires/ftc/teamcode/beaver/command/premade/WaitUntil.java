package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;
import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.beaver.util.Stopwatch;

import java.util.Collections;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoublePredicate;

public class WaitUntil implements Command {
    private BooleanSupplier f1;
    private DoublePredicate f2;

    private Stopwatch stopwatch;

    public WaitUntil(BooleanSupplier f) {
        this.f1 = f;
    }

    public WaitUntil(DoublePredicate f) {
        this.f2 = f;
        stopwatch = new Stopwatch();
    }

    @Override
    public Set<Subsystem> getImmediateDependencies() { return Collections.emptySet(); }

    @Override
    public void start() {
        if (stopwatch != null) stopwatch.reset();
    }

    @Override
    public boolean periodic() {
        if (f1 != null) return f1.getAsBoolean();
        else return f2.test(stopwatch.getElapsed());
    }
}
