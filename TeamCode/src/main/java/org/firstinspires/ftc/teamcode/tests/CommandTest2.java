package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Sequential;
import org.firstinspires.ftc.teamcode.tests.utils.FakeCommand;

import java.util.HashSet;

@Autonomous(group = "Tests")
public final class CommandTest2 extends CommandRuntimeOpMode {
    FakeCommand command1 = new FakeCommand(3, new HashSet<>(), "Command 1");
    FakeCommand command2 = new FakeCommand(3, new HashSet<>(), "Command 2");

    int iteration = 0;

    @Override
    protected void onInit() {
        RobotLog.d("CommandTest : onInit() called");
    }

    @Override
    protected void periodicInit() {
        RobotLog.d("CommandTest : periodicInit() called");
    }

    @Override
    protected void onStart() {
        RobotLog.d("CommandTest : onStart() called");
        iteration = 0;
        schedule(
                new Sequential(command1, command2, new Instant(() -> requestOpModeStop()))
        );
    }

    @Override
    protected void periodic() {
        RobotLog.d("CommandTest : periodic() called");
        iteration++;
    }

    @Override
    protected void onStop() {
        RobotLog.d("CommandTest : onStop() called");
    }
}
