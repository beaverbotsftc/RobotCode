package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.tests.utils.FakeCommand;
import org.firstinspires.ftc.teamcode.tests.utils.FakeSubsystem;

import java.util.HashSet;
import java.util.Set;

@Autonomous(group = "Tests")
public final class CommandTest extends CommandRuntimeOpMode {
    FakeSubsystem subsystem1 = new FakeSubsystem(new HashSet<>(), "Subsystem 1");
    FakeSubsystem subsystem2 = new FakeSubsystem(new HashSet<>(), "Subsystem 2");
    FakeSubsystem subsystem3 = new FakeSubsystem(Set.of(subsystem1), "Subsystem 3");
    FakeSubsystem subsystem4 = new FakeSubsystem(Set.of(subsystem2), "Subsystem 4");
    FakeSubsystem subsystem5 = new FakeSubsystem(Set.of(subsystem2), "Subsystem 5");

    FakeCommand command1 = new FakeCommand(3, new HashSet<>(), "Command 1");
    FakeCommand command2 = new FakeCommand(10,new HashSet<>(),  "Command 2");
    FakeCommand command3 = new FakeCommand(5, Set.of(subsystem1), "Command 3");
    FakeCommand command4 = new FakeCommand(5, Set.of(subsystem2), "Command 4");
    FakeCommand command5 = new FakeCommand(5, Set.of(subsystem3, subsystem4), "Command 5");

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
        schedule(command1, command2, command3, command4, command5);
    }

    @Override
    protected void periodic() {
        RobotLog.d("CommandTest : periodic() called");
        if (iteration != 0) {
            assert iteration > 3 ^ getRunningCommands().contains(command1);
            assert iteration > 10 ^ getRunningCommands().contains(command2);
            assert !getRunningCommands().contains(command3);
            assert !getRunningCommands().contains(command4);
            assert iteration > 5 ^ getRunningCommands().contains(command5);
        }
        iteration++;
    }

    @Override
    protected void onStop() {
        RobotLog.d("CommandTest : onStop() called");
    }
}
