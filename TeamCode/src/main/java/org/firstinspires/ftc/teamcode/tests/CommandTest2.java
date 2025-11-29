package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Sequential;
import org.firstinspires.ftc.teamcode.tests.utils.FakeCommand;
import org.firstinspires.ftc.teamcode.tests.utils.FakeSubsystem;

import java.util.HashSet;
import java.util.Set;

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
