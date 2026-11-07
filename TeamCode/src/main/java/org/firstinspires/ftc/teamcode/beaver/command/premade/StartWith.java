package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;
import org.firstinspires.ftc.teamcode.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.beaver.util.Beaver;

import java.util.Collections;
import java.util.Set;

public class StartWith extends  ArbitraryDependenciesCommand {
    private final Command command;
    private final Runnable start;

    public StartWith(Command command, Runnable start, Subsystem... dependencies) {
        super(Beaver.concat(dependencies, command.getImmediateDependencies().toArray(new Subsystem[0])));
        this.command = command;
        this.start = start;
    }

    @Override
    public void start() {
        start.run();
        command.start();
    }

    @Override
    public boolean periodic() {
        return command.periodic();
    }

    @Override
    public void stop() {
        command.stop();
    }
}
