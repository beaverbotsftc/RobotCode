package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.ConcurrentCommandGroup;
import org.beaverbots.BeaverCommand.ConflictingCommandsException;
import org.beaverbots.BeaverCommand.SequentialCommandGroup;
import org.beaverbots.BeaverCommand.Subsystem;

import java.util.Set;

@TeleOp
public class CommandTest extends CommandRuntimeOpMode {
    private static class SubsystemA implements Subsystem {}
    private static class SubsystemB implements Subsystem {}
    private static class SharedSubsystem implements Subsystem {}

    private static class FinishingCommand implements Command {
        private final String name;
        private final Set<Subsystem> dependencies;
        private final int cyclesToRun;
        private int runCount = 0;

        public FinishingCommand(String name, Set<Subsystem> dependencies, int cyclesToRun) {
            this.name = name;
            this.dependencies = dependencies;
            this.cyclesToRun = cyclesToRun;
        }

        @Override
        public Set<Subsystem> getDependencies() { return dependencies; }

        @Override
        public void start() {
            runCount = 0;
            RobotLog.i("Command '%s' started.", name);
        }

        @Override
        public boolean periodic() {
            runCount++;
            RobotLog.i("Command '%s' periodic (run %d/%d).", name, runCount, cyclesToRun);
            return runCount >= cyclesToRun;
        }

        @Override
        public void stop() {
            RobotLog.i("Command '%s' stopped (finished).", name);
        }

        @Override
        public String toString() { return name; }
    }

    private SubsystemA subsystemA;
    private SubsystemB subsystemB;
    private SharedSubsystem sharedSubsystem;

    private Command sequentialGroup;
    private Command concurrentGroup;

    private int periodicCount = 0;
    private boolean concurrentTestScheduled = false;

    @Override
    protected void onInit() {
        RobotLog.d("onInit called");

        subsystemA = new SubsystemA();
        subsystemB = new SubsystemB();
        sharedSubsystem = new SharedSubsystem();
        register(subsystemA, subsystemB, sharedSubsystem);

        RobotLog.i("TEST: Verifying ConcurrentCommandGroup constructor conflict detection.");
        try {
            // These two commands conflict because they both require SharedSubsystem.
            Command conflictingCmd1 = new FinishingCommand("ConflictingCmd1", Set.of(sharedSubsystem), 3);
            Command conflictingCmd2 = new FinishingCommand("ConflictingCmd2", Set.of(sharedSubsystem), 3);

            // This constructor call should fail and throw an exception.
            new ConcurrentCommandGroup(conflictingCmd1, conflictingCmd2);
            RobotLog.e("TEST FAILED: ConcurrentCommandGroup did not detect an internal dependency conflict.");
        } catch (ConflictingCommandsException e) {
            RobotLog.i("TEST PASSED: ConcurrentCommandGroup correctly threw ConflictingCommandsException.");
        } catch (Exception e) {
            RobotLog.e("TEST FAILED: An unexpected exception occurred during ConcurrentCommandGroup test: " + e.getMessage());
        }


        // A sequential group where two commands share the same subsystem. This is valid.
        // Command execution order: A -> B -> C. Total runtime: 3 + 4 + 2 = 9 cycles.
        sequentialGroup = new SequentialCommandGroup(
                new FinishingCommand("SeqA (Shared)", Set.of(sharedSubsystem), 3),
                new FinishingCommand("SeqB (Independent)", Set.of(subsystemA), 4),
                new FinishingCommand("SeqC (Shared)", Set.of(sharedSubsystem), 2)
        );

        // A concurrent group where commands have independent subsystems.
        // Command "Short" runs for 4 cycles, "Long" runs for 12.
        // The group should run for a total of 12 cycles.
        concurrentGroup = new ConcurrentCommandGroup(
                new FinishingCommand("ConcShort (SubA)", Set.of(subsystemA), 4),
                new FinishingCommand("ConcLong (SubB)", Set.of(subsystemB), 12)
        );

        RobotLog.i("Initialization complete. Runtime tests will begin on start.");
    }


    @Override
    protected void onStart() {
        RobotLog.d("onStart called");
        RobotLog.w("RUNTIME TEST 1: Scheduling SequentialCommandGroup.");
        RobotLog.w("EXPECTED: SeqA runs for 3 loops, then SeqB for 4, then SeqC for 2.");
        schedule(sequentialGroup);
    }

    @Override
    protected void periodic() {
        RobotLog.d("periodic called (count: " + periodicCount + ")");
        periodicCount++;

        // After 15 cycles, the sequential group should be long finished.
        // We can now start the next test.
        if (periodicCount > 15 && !concurrentTestScheduled) {
            concurrentTestScheduled = true;
            RobotLog.w("RUNTIME TEST 2: Scheduling ConcurrentCommandGroup.");
            RobotLog.w("EXPECTED: ConcShort and ConcLong start together. Group finishes after ConcLong (12 loops).");
            schedule(concurrentGroup);
        }

        telemetry.update();
    }

    @Override
    protected void onStop() {
        RobotLog.d("onStop called");
    }
}