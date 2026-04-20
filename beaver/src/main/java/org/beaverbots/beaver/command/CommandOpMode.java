package org.beaverbots.beaver.command;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public abstract class CommandOpMode extends OpMode {
    private final List<Command> commandBuffer = new ArrayList<>();
    private final Set<Subsystem> usedSubsystems = new HashSet<>();
    private final List<Subsystem> registeredSubsystems = new ArrayList<>();

    private final List<Command> commandsToSchedule = new ArrayList<>();
    private final Set<Command> commandsToCancel = new HashSet<>();

    private List<LynxModule> hubs;

    public static long loopNumber = 0;
    private int telemetryUpdateFrequency = 1;
    private int telemetryUpdateOffset = 1;
    private int[] lynxUpdateFrequency = new int[] {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    private int[] lynxUpdateOffset = new int[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


    private FtcDashboard dashboardInstance;

    public static TelemetryPacket packet;
    public static Telemetry telemetryInstance;

    protected final void setTelemetryUpdateFrequency(int telemetryUpdateFrequency, int telemetryUpdateOffset) {
        this.telemetryUpdateFrequency = telemetryUpdateFrequency;
        this.telemetryUpdateOffset = telemetryUpdateOffset;
    }

    protected final void setLynxUpdateFrequency(int[] lynxUpdateFrequency, int[] lynxUpdateOffset) {
        this.lynxUpdateFrequency = lynxUpdateFrequency;
        this.lynxUpdateOffset = lynxUpdateOffset;
    }

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

            // Technically, if it were canceled, you don't need to add them. However, it makes it easier to reason about.
            // Also, the only way this could cause a problem is if multiple commands are scheduled within the same loop.
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
        loopNumber = 0;

        dashboardInstance = FtcDashboard.getInstance();
        telemetryInstance = telemetry;

        packet = new TelemetryPacket();

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        HardwareManager.init(hardwareMap);
        onInit();

        telemetry.update();
        dashboardInstance.sendTelemetryPacket(packet);
    }

    public final void init_loop() {
        packet = new TelemetryPacket();
        telemetry.clear();

        for (int i = 0; i < hubs.size(); i++)
            if (loopNumber == 0 || (loopNumber + lynxUpdateOffset[i]) % lynxUpdateFrequency[i] == 0)
                hubs.get(i).clearBulkCache();

        runSubsystems();
        periodicInit();
        runScheduler();

        if ((loopNumber + telemetryUpdateOffset) % telemetryUpdateFrequency == 0) {
            dashboardInstance.sendTelemetryPacket(packet);
            telemetry.update();
        }

        loopNumber++;
    }

    public final void start() {
        onStart();
    }

    public final void loop() {
        packet = new TelemetryPacket();
        telemetry.clear();

        for (int i = 0; i < hubs.size(); i++)
            if (loopNumber == 0 || (loopNumber + lynxUpdateOffset[i]) % lynxUpdateFrequency[i] == 0)
                hubs.get(i).clearBulkCache();

        runSubsystems();
        periodic();
        runScheduler();

        if ((loopNumber + telemetryUpdateOffset) % telemetryUpdateFrequency == 0) {
            dashboardInstance.sendTelemetryPacket(packet);
            telemetry.update();
        }

        loopNumber++;
    }

    public final void stop() {
        cancelAll();
        runScheduler(); // To process cancellations
        onStop();
        HardwareManager.reset();
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

    protected final void unregister(Subsystem... subsystems) {
        RobotLog.dd("BeaverCommand", "Subsystem(s) being unregistered: %s", Arrays.toString(subsystems));
        this.registeredSubsystems.removeAll(Arrays.asList(subsystems));
    }

    protected final Set<Command> getRunningCommands() {
        return new HashSet<>(commandBuffer);
    }

    protected final boolean isRunning(Command command) {
        return getRunningCommands().contains(command);
    }

    protected void onInit() {}
    protected void periodicInit() {}
    protected void onStart() {}
    protected void periodic() {}
    protected void onStop() {}

    public static void addLine(String line) {
        packet.addLine(line);
        telemetryInstance.addLine(line);
    }

    public static void addData(String tag, Object data) {
        packet.put(tag, data);
        telemetryInstance.addData(tag, data);
    }
}