package org.firstinspires.ftc.teamcode.subsystems.intake;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public final class IntakeMove implements Command {
    private Intake intake;
    private Gamepad gamepad;

    public IntakeMove(Intake intake, Gamepad gamepad) {
        this.intake = intake;
        this.gamepad = gamepad;
    }

    @Override
    public Set<Subsystem> getDependencies() {
        return new HashSet<>(Collections.singletonList(intake));
    }

    @Override
    public boolean periodic() {
        intake.move(gamepad.getRightY());
        return false;
    }
}