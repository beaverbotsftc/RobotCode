package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public final class ControllerMove implements Command {
    private Drivetrain drivetrain;
    private Gamepad gamepad;

    public ControllerMove(Drivetrain drivetrain, Gamepad gamepad) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;
    }

    @Override
    public Set<Subsystem> getDependencies() {
        return new HashSet<>(Collections.singletonList(drivetrain));
    }

    @Override
    public boolean periodic() {
        double x = gamepad.getLeftX();
        double y = gamepad.getLeftY();
        double theta = gamepad.getRightX();

        drivetrain.setMotion(x, y, theta);

        return false;
    }
}
