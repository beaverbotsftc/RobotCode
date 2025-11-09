package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.Subsystem;
import org.firstinspires.ftc.teamcode.DrivetrainState;
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
        double x = gamepad.getLeftY();
        double y = -gamepad.getLeftX();
        double theta = -gamepad.getRightX();

        drivetrain.move(new DrivetrainState(x, y, theta));

        return false;
    }
}
