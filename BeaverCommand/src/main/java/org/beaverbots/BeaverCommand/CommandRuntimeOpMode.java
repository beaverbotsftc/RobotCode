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
    private List<Command> canonicalCommandBuffer = new ArrayList<>();
    private List<Command> updatedCommandBuffer = new ArrayList<>();
    private final List<Subsystem> subsystems = new ArrayList<>();
    private Set<Subsystem> canonicalUsedSubsystems = new HashSet<>();
    private Set<Subsystem> updatedUsedSubsystems = new HashSet<>();

    // This just removes all earlier conflicting dependencies
    private void solveDependencyConflicts() {
        List<Command> commandBuffer = new ArrayList<>();
        Set<Subsystem> usedSubsystems = new HashSet<>();

        // Iterate over the command buffer in reverse
        // This is fine because it will be overwritten at the end of the function
        Collections.reverse(updatedCommandBuffer);
        for (Command command : updatedCommandBuffer) {
            Set<Subsystem> dependencies = Command.calculateDependencies(command);
            if (Collections.disjoint(dependencies, usedSubsystems)) {
                RobotLog.dd("BeaverCommand", "Keeping command '%s' in the command buffer; it has no dependency conflicts.", command);
                commandBuffer.add(command);
                usedSubsystems.addAll(dependencies);
            } else {
                RobotLog.dd("BeaverCommand", "Removing command '%s' from the command buffer; it has dependency conflicts.", command);
            }
        }

        // The updated command buffer was built in reverse, so this will undo that
        Collections.reverse(commandBuffer);

        updatedCommandBuffer = commandBuffer;
        updatedUsedSubsystems = usedSubsystems;
    }

    private void runAllCommands() {
        List<Command> commandBuffer = new ArrayList<>();
        for (Command command : this.canonicalCommandBuffer) {
            RobotLog.dd("BeaverCommand", "Running periodic() on command '%s'", command);
            if (!command.periodic()) {
                // It should remain in the command buffer, because it isn't finished yet
                RobotLog.dd("BeaverCommand", "Keeping command '%s' in the command buffer; it didn't finish yet.", command);
                commandBuffer.add(command);
            } else {
                // It should be removed from the command buffer due to the command finishing
                RobotLog.dd("BeaverCommand", "Removing command '%s' from the command buffer; it finished.", command);
                RobotLog.dd("BeaverCommand", "Running stop() on command '%s'", command);
                command.stop();
            }
        }
        // Dependency logic
        updatedCommandBuffer = commandBuffer;
    }

    private void runAllSubsystems() {
        for (Subsystem subsystem : subsystems) {
            RobotLog.dd("BeaverCommand", "Running periodic() on subsystem: '%s'", subsystem);
            subsystem.periodic();
        }
    }

    private void runAllSubsystemsDefaults() {
        for (Subsystem subsystem : subsystems) {
            if (!canonicalUsedSubsystems.contains(subsystem)) {
                RobotLog.dd("BeaverCommand", "Running periodicDefault() on subsystem: '%s', it's not being used.", subsystem);
                subsystem.periodicDefault();
            } else {
                RobotLog.dd("BeaverCommand", "Not running periodicDefault() on subsystem: '%s', it's being used.", subsystem);
            }
        }
    }

    private void updateState() {
        canonicalCommandBuffer = updatedCommandBuffer;
        canonicalUsedSubsystems = updatedUsedSubsystems;
    }

    public final void init() {
        HardwareManager.init(hardwareMap);
        onInit();
    }
    public final void init_loop() {
        runAllSubsystems();
        periodicInit();
        updateState();
        runAllCommands();
        runAllSubsystemsDefaults();
    }
    public final void start() {
        onStart();
    }
    public final void loop() {
        runAllSubsystems();
        periodic();
        updateState();
        runAllCommands();
        runAllSubsystemsDefaults();
    }
    public final void stop() {
        onStop();
    }

    ///  Note that this does *not* solve dependency conflicts. For that, use the public schedule function.
    private void scheduleIndividual(Command command) {
        RobotLog.dd("BeaverCommand", "Command '%s' being scheduled.", command);
        updatedCommandBuffer.add(command);
        RobotLog.dd("BeaverCommand", "Running start() on command '%s'", command);
        command.start();
    }

    /// Should any command's dependencies conflict with any others', the one added last
    /// will take precedence, and the earlier ones will be stopped.
    protected final void schedule(Command... commands) {
        for (Command command : commands) {
            scheduleIndividual(command);
        }
        solveDependencyConflicts();
    }

    /// In an effort to decrease bugs and increase determinism, subsystem periodic() methods
    /// are (concurrently) executed in the order they were registered.
    /// However, relying on this is fragile and not good practice.
    protected final void register(Subsystem... subsystems) {
        RobotLog.dd("BeaverCommand", "Subsystem(s) being registered: %s", Arrays.toString(subsystems));
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    protected void onInit() {}
    protected void periodicInit() {}
    protected void onStart() {}
    protected void periodic() {}
    protected void onStop() {}
}
