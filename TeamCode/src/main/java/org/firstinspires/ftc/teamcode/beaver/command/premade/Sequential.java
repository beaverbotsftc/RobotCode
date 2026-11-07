package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;

import java.util.function.IntPredicate;

public class Sequential extends Cycle {
    public Sequential(Command... commands) {
        super(i -> i >= 1, commands);
    }

    private Sequential(IntPredicate isFinished, Command... commands) {}
}