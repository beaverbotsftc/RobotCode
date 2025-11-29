package org.beaverbots.BeaverCommand;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public abstract class CommandRuntimeOpMode extends OpMode {
    private final List<Command> commandBuffer = new ArrayList<>();
    private final Set<Subsystem> usedSubsystems = new HashSet<>();
    private final List<Subsystem> registeredSubsystems = new ArrayList<>();

    private final List<Command> commandsToSchedule = new ArrayList<>();
    private final Set<Command> commandsToCancel = new HashSet<>();

    private boolean isInitialized = false;

    private void runScheduler() {
        for (Command command : commandBuffer) {
            if (!commandsToCancel.contains(command) && command.periodic()) {
                RobotLog.dd("BeaverCommand", String.format("Command '%s' finished.", command));
                commandsToCancel.add(command);
            }
        }

        for (Command command : commandsToCancel) {
            // Only stop if it's actually running and wasn't just queued for schedule then cancelled.
            if (commandBuffer.contains(command)) {
                RobotLog.dd("BeaverCommand", String.format("Running stop() on command '%s' due to cancellation or finish.", command));
                command.stop();
                commandBuffer.remove(command);
            }
        }

        for (Command command : commandsToSchedule) {
            if (commandsToCancel.contains(command)) continue;
            RobotLog.dd("BeaverCommand", String.format("Running start() on newly scheduled command '%s'", command));
            command.start();
            commandBuffer.add(command);
        }

        commandsToSchedule.clear();
        commandsToCancel.clear();

        Set<Subsystem> conflictingSubsystems = new HashSet<>();
        for (int i = commandBuffer.size() - 1; i >= 0; i--) {
            Command command = commandBuffer.get(i);
            Set<Subsystem> dependencies = Command.calculateDependencies(command);

            if (!Collections.disjoint(dependencies, conflictingSubsystems)) {
                RobotLog.dd("BeaverCommand", String.format("Cancelling command '%s' due to dependency conflict.", command));
                command.stop();
                commandBuffer.remove(i);
            }

            conflictingSubsystems.addAll(dependencies);
        }

        usedSubsystems.clear();
        for (Command command : commandBuffer) {
            usedSubsystems.addAll(Command.calculateDependencies(command));
        }

        for (Subsystem subsystem : registeredSubsystems) {
            if (!usedSubsystems.contains(subsystem)) {
                subsystem.periodicDefault();
            }
        }
    }

    private void runSubsystems() {
        for (Subsystem subsystem : registeredSubsystems) {
            subsystem.periodic();
        }
    }

    public final void init() {
        HardwareManager.init(hardwareMap);
        onInit();
    }

    public final void init_loop() {
        if (!isInitialized) {
            isInitialized = true;
        }

        runSubsystems();
        periodicInit();
        runScheduler();
        telemetry.update();
    }

    public final void start() {
        onStart();
    }

    public final void loop() {
        runSubsystems();
        periodic();
        runScheduler();
        telemetry.update();
    }

    public final void stop() {
        cancelAll();
        runScheduler(); // To process cancellations
        onStop();
    }

    protected final void schedule(Command... commands) {
        RobotLog.dd("BeaverCommand", String.format("Command(s) being scheduled: %s", Arrays.toString(commands)));
        commandsToSchedule.addAll(Arrays.asList(commands));
    }

    protected final void cancel(Command... commands) {
        RobotLog.dd("BeaverCommand", String.format("Command(s) being cancelled: %s", Arrays.toString(commands)));
        commandsToCancel.addAll(Arrays.asList(commands));
    }

    protected final void cancelAll() {
        RobotLog.dd("BeaverCommand", "Cancelling all commands");
        commandsToCancel.addAll(commandBuffer);
        commandsToSchedule.clear(); // Also clear any pending commands that haven't started.
    }

    protected final void register(Subsystem... subsystems) {
        RobotLog.dd("BeaverCommand", "Subsystem(s) being registered: %s", Arrays.toString(subsystems));
        this.registeredSubsystems.addAll(Arrays.asList(subsystems));
    }

    protected final Set<Command> getRunningCommands() {
        return new HashSet<>(commandBuffer);
    }

    protected void onInit() {}
    protected void periodicInit() {}
    protected void onStart() {}
    protected void periodic() {}
    protected void onStop() {}
}