package org.firstinspires.ftc.teamcode.beaver.command;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.beaver.command.premade.GamepadEx;
import org.firstinspires.ftc.teamcode.beaver.util.Stopwatch;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public abstract class CommandOpMode extends OpMode {
    private final List<Command> commandBuffer = new ArrayList<>();
    private final List<Subsystem> subsystems = new ArrayList<>();

    private final List<Command> commandsToSchedule = new ArrayList<>();
    private final List<Command> commandsToCancel = new ArrayList<>();

    public static long loopNumber = 0;

    protected static int telemetryUpdateWait = 1;
    protected static int telemetryUpdateOffset = 0;

    public static List<String> telemetryLines = new ArrayList<>();

    public static final boolean PANELS = true;
    private Telemetry panelsTelemetry;

    protected GamepadEx gamepad;

    private static final Stopwatch stopwatch = new Stopwatch();
    private static double dt;
    private static double averageDt;
    private static double elapsed;

    public static double getDt() {
        return dt;
    }

    public static double getAverageDt() {
        return averageDt;
    }

    public static double getElapsed() {
        return elapsed;
    }

    public static boolean checkLoop(int modulo, int offset) {
        return (loopNumber - offset) % modulo == 0;
    }

    public static boolean isTelemetryLoop() {
        return checkLoop(telemetryUpdateWait, telemetryUpdateOffset);
    }

    public static void addLine(String message) {
        if (!isTelemetryLoop()) return;

        telemetryLines.add(message);
    }

    public static void addLine() {
        addLine("");
    }

    /// This can work with primitives too via autoboxing.
    public static void addData(String key, Object value) {
        if (!isTelemetryLoop()) return;

        telemetryLines.add(key + " : " + value);
    }

    public static void addLineUrgent(String subsystem, String message) {
        String formattedMessage = "[" + subsystem + "] " + message + " @ " + System.nanoTime();
        telemetryLines.add(formattedMessage);
        RobotLog.ww(subsystem, message);
    }

    private void updateTelemetry() {
        for (String line : telemetryLines) {
            telemetry.addLine(line);
            PanelsTelemetry.INSTANCE.getTelemetry().addLine(line);
        }
        telemetry.update();
        if (PANELS)
            panelsTelemetry.update();
        telemetryLines.clear();
    }

    protected final void register(Subsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    protected final void schedule(Command... commands) {
        commandsToSchedule.addAll(Arrays.asList(commands));
    }

    protected final void cancel(Command... commands) {
        commandsToCancel.addAll(Arrays.asList(commands));
    }

    private void runSubsystems() {
        for (Subsystem subsystem : subsystems)
            subsystem.periodic();
    }

    private void runScheduler() {
        for (Command command : commandBuffer)
            if (!commandsToCancel.contains(command) && command.periodic())
                commandsToCancel.add(command);

        for (Command command : commandsToCancel)
            // Only stop if it's actually running and wasn't just queued for schedule then cancelled.
            if (commandBuffer.contains(command)) {
                RobotLog.dd("BeaverCommand", "Stopping command " + command);
                command.stop();
                commandBuffer.remove(command);
            }

        // This works if subsystems reported by commands never change and if commands scheduled within one loop never
        Set<Subsystem> usedSubsystems = new HashSet<>();
        for (Command command : commandsToSchedule)
            usedSubsystems.addAll(Command.calculateDependencies(command));
        for (int i = 0; i < commandBuffer.size(); i++) {
            Command command = commandBuffer.get(i);
            Set<Subsystem> dependencies = Command.calculateDependencies(command);

            if (!Collections.disjoint(dependencies, usedSubsystems)) {
                RobotLog.dd("BeaverCommand", "Cancelling command " + command + " due to dependency conflict");
                command.stop();
                commandBuffer.remove(i);
                i--;
            }
        }

        // This could cause problems if conflicting things are scheduled in the same tick,
        // but I'll assume that doesn't happen.
        for (Command command : commandsToSchedule) {
            if (commandsToCancel.contains(command)) continue;
            RobotLog.dd("BeaverCommand", "Starting command " + command);
            command.start();
            commandBuffer.add(command);
        }

        commandsToSchedule.clear();
        commandsToCancel.clear();
    }

    protected void onInit() {}
    protected void initPeriodic() {}
    protected void onStart() {}
    protected void periodic() {}
    protected void onStop() {}
    protected double getThisAverageDt() {
        return 1.0 / 20.0;
    }

    @Override
    public final void init() {
        if (PANELS)
            panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        loopNumber = 0;
        telemetryLines.clear();

        telemetry.setMsTransmissionInterval(0);
        if (PANELS)
            panelsTelemetry.setMsTransmissionInterval(0);
        PanelsTelemetry.INSTANCE.getTelemetry().update();

        HardwareManager.init(hardwareMap);

        gamepad = new GamepadEx(gamepad1);
        register(gamepad);

        stopwatch.reset();
        stopwatch.getDt();
        averageDt = getThisAverageDt();
        dt = averageDt;
        elapsed = 0;
        onInit();
        updateTelemetry();
    }
    @Override
    public final void init_loop() {
        dt = stopwatch.getDt();
        elapsed = stopwatch.getElapsed();
        runSubsystems();
        initPeriodic();
        runScheduler();
        if (isTelemetryLoop()) updateTelemetry();
        loopNumber++;
    }

    @Override
    public final void start() {
        dt = stopwatch.getDt();
        elapsed = stopwatch.getElapsed();
        onStart();
        updateTelemetry();
    }
    @Override
    public final void loop() {
        dt = stopwatch.getDt();
        elapsed = stopwatch.getElapsed();
        runSubsystems();
        periodic();
        runScheduler();
        if (isTelemetryLoop()) updateTelemetry();
        loopNumber++;
    }
    @Override
    public final void stop() {
        dt = stopwatch.getDt();
        elapsed = stopwatch.getElapsed();
        onStop();
        updateTelemetry();
    }

}
